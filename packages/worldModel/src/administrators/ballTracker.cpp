 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ballTracker.cpp
 *
 *  Created on: Sep 6, 2016
 *      Author: Tim Kouters
 */

#include "int/administrators/ballTracker.hpp"

#include "int/configurators/ballTrackerConfigurator.hpp"

#include "cDiagnosticsEvents.hpp"

size_t ballTracker::_staticTrackerID = 0;

void ballTracker::reset()
{
    _staticTrackerID = 0;
}

ballTracker::ballTracker(const objectMeasurementCache &measurement)
/*!
 * \brief Top-level ball tracker that calculates the ball position
 *
 * This class functions as a shell for future algorithms to track the ball
 *
 */
{
	_ballMeasurements.push_back(measurement);
	_staticTrackerID++;
	_trackerID = _staticTrackerID;
	_lastBallResult.setId(_trackerID);

    // initialize ballResult so new measurements can be added w.r.t. its position
	Vector3D pos = _ballMeasurements.back().getPositionFcs();
    _lastBallResult.setCoordinates(pos.x, pos.y, pos.z);
    
    // store timestamp of creation for age calculation
    _t0 = measurement.getObjectMeasurement().getTimestamp();
    _good = false;
    
    // get and store once all confidence configuration parameters
    // TODO make static? then take care of re-inits
    _ballCfg.fitConfig.measPerOrder = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorIntegers::measPerOrder);
    _ballCfg.fitConfig.groupingDt = ballTrackerConfigurator::getInstance().get    
(ballTrackerConfiguratorFloats::groupingDt);
    _ballCfg.fitConfig.outlierNSigma = ballTrackerConfigurator::getInstance().get    
(ballTrackerConfiguratorFloats::outlierNSigma);
    _ballCfg.fitConfig.outlierMaxIter = ballTrackerConfigurator::getInstance().get    
(ballTrackerConfiguratorIntegers::outlierMaxIter);
    _ballCfg.fitConfig.outlierIterFraction = ballTrackerConfigurator::getInstance().get    
(ballTrackerConfiguratorFloats::outlierIterFraction);
    _ballCfg.fitConfig.depthWeight = ballTrackerConfigurator::getInstance().get    
(ballTrackerConfiguratorFloats::solverCoreWeight);
    _ballCfg.fitConfig.speedFitOrder = (int)ballTrackerConfigurator::getInstance().get    
(ballTrackerConfiguratorBool::solverCoreSpeed);
    _ballCfg.numCams = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorIntegers::confidenceNumCams);
    _ballCfg.measLim = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorIntegers::confidenceMeasLim);
    _ballCfg.omniPref = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorBool::confidenceOmniPref);
    _ballCfg.freshLim = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::confidenceFreshLim);
    _ballCfg.ageLim = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::confidenceAgeLim);
    _ballCfg.timeout = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::solverTrackerTimeout);
    _ballCfg.fitLim1 = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::confidenceFitLim1);
    _ballCfg.fitLim2 = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::confidenceFitLim2);
    _ballCfg.zLim1 = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::confidenceZLim1);
    _ballCfg.zLim2 = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::confidenceZLim2);
    _ballCfg.vLim1 = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::confidenceVLim1);
    _ballCfg.vLim2 = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::confidenceVLim2);
    _ballCfg.useFrontVision = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorBool::useFrontVision);
    _ballCfg.useFriendlyMeas = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorBool::useFriendlyMeas);
    _ballCfg.friendlyMeasurementsDistance = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::friendlyMeasurementsDistance);
    
    ballTrackerConfigurator::getInstance().traceAll();
    
    _tracker.setConfig(_ballCfg.fitConfig);
}

ballTracker::~ballTracker()
/*
 * Chuck Norris doesn't wear a watch.
 * He decides what time it is.
 */
{
	_ballMeasurements.clear();
}

bool sortOnIncreasingTime(const objectMeasurementCache& objA, const objectMeasurementCache& objB)
{
    return objA.getObjectMeasurement().getTimestamp() < objB.getObjectMeasurement().getTimestamp();
}

void ballTracker::addBallMeasurement(const objectMeasurementCache &measurement, bool &measurementIsAdded)
{
	try
	{
		/*
		 * Here we either accept or reject measurement for this tracker object
		 * Current method: 
		 *  - take position from latest solution 
		 *    (if not available yet, which happens a brief moment after creation, then take latest measurement)
		 *  - check distance against threshold  
		 */
		 
        // TODO if we use frontVision, then distance can hardly be trusted. Accept/reject should not be done
        // based on XY distance, but instead based on angles?
        
		Vector3D currentPos(_lastBallResult.getX(), _lastBallResult.getY(), _lastBallResult.getZ());
		Vector3D newPos = measurement.getPositionFcs();
        float threshold = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::solverTrackerXYTolerance);
		if (vectorsize(currentPos - newPos) < threshold)
		{
            // as far as discriminator is concerned, this measurement will now be processed
            // in principle it is added and to be used, but it may also be the case that we here already reject it
            // in which case the measurement is not added to the vector, but flag measurementIsAdded is set
            // TODO: make this nicer (rename measurementIsAdded to measurementIsProcessed? filter elsewhere?)
            bool accept = true;
            if (!_ballCfg.useFrontVision && (measurement.getObjectMeasurement().getCameraType() == cameraType::FRONTCAMERA))
            {
                accept = false;
            }
            if (!_ballCfg.useFriendlyMeas && (measurement.getObjectMeasurement().getID().robotID != getRobotNumber()))
            {
                accept = false;
            }
		    
            if (accept)
            {
                // invariant: measurements shall be sorted by time
                // optimization: only sort if needed, i.e. timestamp is not newer than the last one
                if (measurement.getObjectMeasurement().getTimestamp() < _ballMeasurements.back().getObjectMeasurement().getTimestamp())
                {
                    _ballMeasurements.push_back(measurement);
                    std::sort(_ballMeasurements.begin(), _ballMeasurements.end(), sortOnIncreasingTime);
                }
                else
                {
                    _ballMeasurements.push_back(measurement);
                    // no sort required
                }
            }
            
            // inform discriminator that this measurement is processed, stop iterating over trackers
		    measurementIsAdded = true;
        }
        else
        {
		    measurementIsAdded = false;
        }
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void ballTracker::calculateConfidence(double t)
{

    // ported implementation from fbt_tracker_confidence.m
    // confidence is determined by calculating several sub-scores, each between 0.0 (bad) and 1.0 (good)
    // final confidence is the product of each sub-score
    
    // iterate once through all measurements, to extract some values
    double tmin = 1e20, tmax = 0.0;
    _confDetails.withOmni = false;
    std::set<uint8_t> usedCams;
    for (auto it = _ballMeasurements.begin(); it != _ballMeasurements.end(); ++it)
    {
        usedCams.insert(it->getObjectMeasurement().getID().robotID);
        if (it->getObjectMeasurement().getCameraType() == cameraType::OMNIVISION)
        {
            _confDetails.withOmni = true;
        }
        tmin = std::min(tmin, it->getObjectMeasurement().getTimestamp());
        tmax = std::max(tmax, it->getObjectMeasurement().getTimestamp());
    }
    
    // camera score (more is better, omnivision beats frontvision)
    _confDetails.numCams = 0;
    _confDetails.numCams = usedCams.size();
    float camScale = _ballCfg.numCams * (1 + _ballCfg.omniPref);
    _confDetails.camScore = std::min(_confDetails.numCams, _ballCfg.numCams) * (1 + (_confDetails.withOmni && _ballCfg.omniPref)) / camScale;

    // number of measurements (more is better)
    _confDetails.measScore = std::min((int)_ballMeasurements.size(), _ballCfg.measLim) / (1.0 * _ballCfg.measLim);

    // age (higher is better) and freshness (time since last measurement - lower is better)
    _confDetails.ageScore = std::min(1.0, (tmax - _t0) / _ballCfg.ageLim);
    _confDetails.freshScore = 1.0 - std::max(0.0, (t - tmax - _ballCfg.freshLim) / (_ballCfg.timeout - _ballCfg.freshLim));

    // fit quality (lower is better)
    _confDetails.fitScore   = 1.0f - std::min(std::max((_confDetails.fitQuality - _ballCfg.fitLim1) / (_ballCfg.fitLim2 - _ballCfg.fitLim1), 0.0f), 1.0f);
    
    // velocity (lower is better)
    Vector3D velocity(_lastBallResult.getVX(), _lastBallResult.getVY(), _lastBallResult.getVZ());
    _confDetails.vScore     = 1.0f - std::min(std::max((vectorsize(velocity) - _ballCfg.vLim1) / (_ballCfg.vLim2 - _ballCfg.vLim1), 0.0f), 1.0f);
    
    // ball height (lower is better)
    _confDetails.zScore     = 1.0f - std::min(std::max((_lastBallResult.getZ() - _ballCfg.zLim1) / (_ballCfg.zLim2 - _ballCfg.zLim1), 0.0f), 1.0f);
    
     // multiply all sub-scores to obtain final confidence
    float confidence = _confDetails.camScore * _confDetails.measScore * _confDetails.ageScore 
                     * _confDetails.freshScore * _confDetails.vScore * _confDetails.zScore
                     * _confDetails.fitScore;
                     
    // good enough?
    float threshold = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::confidenceGoodLimit);
    _good = (confidence > threshold);
    // store
    _lastBallResult.setConfidence(confidence);
    _lastBallResult.setIsValid(_good);
}

void ballTracker::calculateBall(double timestampNow)
{
	try
	{
		// entry point of the algorithm
        _tracker.solve(_ballMeasurements, timestampNow, true);
        // store ball result
        _lastBallResult = ballClass_t(_tracker.getResult());
    	_lastBallResult.setId(_trackerID);
        // calculate confidence and extract some diagnostics values
        _confDetails.fitQuality = _tracker.getFitResidual();
        calculateConfidence(timestampNow);
        // floor clipping
        if (_lastBallResult.getZ() < MIN_BALL_HEIGHT)
        {
            // warn only if this ball had a high confidence and was significantly low in the ground
            if ((_lastBallResult.getConfidence() > MIN_WARNING_CONFIDENCE) && (_lastBallResult.getZ() < MIN_WARNING_HEIGHT))
            {
                TRACE("warning: clipping ball to floor (z=%6.2f, conf=%6.2f)", _lastBallResult.getZ(), _lastBallResult.getConfidence());
                // TODO: give a penalty to the confidence, say 20%?
            }
    		_lastBallResult.setCoordinates(_lastBallResult.getX(), _lastBallResult.getY(), 0.0);
        }
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

bool ballTracker::isTimedOut(double t)
{
	try
	{
		bool retVal = false;

		/*
		 * State when tracker is not longer valid
		 * Current behavior: when list of measurements are empty
		 */
		cleanUpTimedOutBallMeasurements(t);
		retVal = _ballMeasurements.empty();

		return retVal;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

ballClass_t ballTracker::getBall() const
{
	try
	{
		return _lastBallResult;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void ballTracker::cleanUpTimedOutBallMeasurements(double t)
{
	try
	{
		auto trackerTimeout = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::solverTrackerTimeout);

		for(auto i = _ballMeasurements.begin(); i != _ballMeasurements.end(); )
		{
			double time_diff = t - i->getObjectMeasurement().getTimestamp();
			if(time_diff > trackerTimeout)
            {
                // Delete ball measurement
				i = _ballMeasurements.erase(i);
            }
            else
            {
                ++i;
            }
		}
 	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void ballTracker::setGood()
{
    _good = true;
    _lastBallResult.setIsValid(_good);
}

bool ballTracker::good() const
{
    return _good;
}

void ballTracker::traceMeasurements() const
{
    for (auto imeas = _ballMeasurements.begin(); imeas != _ballMeasurements.end(); ++imeas)
    {
        objectMeasurementType m = imeas->getObjectMeasurement();
        Vector3D ballPos = imeas->getPositionFcs();
        TRACE("measurement:  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f",
            m.getCameraX(), m.getCameraY(), m.getCameraZ(), m.getCameraPhi(),
            m.getAzimuth(), m.getElevation(), m.getRadius(), m.getConfidence(),
            ballPos.x, ballPos.y, ballPos.z); 
    }
}

std::string ballTracker::toStr(double tcurr, bool details)
{
    char buf[256];
    double age = tcurr - _t0;
    sprintf(buf, "%4d %4.2f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %5.1f %5d", 
            (int)_trackerID, _lastBallResult.getConfidence(), 
            _lastBallResult.getX(), _lastBallResult.getY(), _lastBallResult.getZ(), 
            _lastBallResult.getVX(), _lastBallResult.getVY(), _lastBallResult.getVZ(), 
            age, (int)_ballMeasurements.size());
    std::string result = buf;
    if (details)
    {
        sprintf(buf, "%5d %4d %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f",
                _confDetails.numCams, _confDetails.withOmni, 
                _confDetails.camScore, _confDetails.ageScore, _confDetails.measScore, _confDetails.freshScore,
                _confDetails.zScore, _confDetails.vScore,
                _confDetails.fitScore, _confDetails.fitQuality);
        result += buf;
    }
    return result;
}

