 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
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

#include "cDiagnostics.hpp"
#include "cEnvironmentField.hpp" // field dimensions
#include "tracing.hpp"

size_t ballTracker::_staticTrackerID = 0;

void ballTracker::reset()
{
    _staticTrackerID = 0;
}

ballTracker::ballTracker(const objectMeasurementCache &measurement)
    : _confDetails()
/*!
* \brief Top-level ball tracker that calculates the ball position
*
* This class functions as a shell for future algorithms to track the ball
*
*/
{
    _ballMeasurements.push_back(measurement);
    _ownBallsFirst = false;
    _staticTrackerID++;
    _trackerID = _staticTrackerID;
    _lastBallResult.setId(_trackerID);

    // initialize ballResult so new measurements can be added w.r.t. its position
    Vector3D pos = _ballMeasurements.back().getPositionFcs();
    _lastBallResult.setCoordinates(pos.x, pos.y, pos.z);
    _lastBallResult.setVelocities(0.0, 0.0, 0.0);

    // store timestamp of creation for age calculation
    _t0 = measurement.getObjectMeasurement().timestamp;
    _lastBallResult.setTimestamp(_t0);
    _blackList = blackListType::UNKNOWN;
    _lastGroundTimeStamp = _t0;
    _lastAirTimeStamp = _t0;
    _tmin = _t0;
    _tmax = _t0;

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
    _ballCfg.fitConfig.minVmeas = (int)ballTrackerConfigurator::getInstance().get    
        (ballTrackerConfiguratorIntegers::solverCoreMinVmeas);
    _ballCfg.numCams = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorIntegers::confidenceNumCams);
    _ballCfg.measLim = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorIntegers::confidenceMeasLim);
    _ballCfg.omniPref = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorBool::confidenceOmniPref);
    _ballCfg.freshLim = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::confidenceFreshLim);
    _ballCfg.ageLim = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::confidenceAgeLim);
    _ballCfg.timeout = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::solverTrackerTimeout);
    _ballCfg.buffer = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::solverTrackerBuffer);
    _ballCfg.fitLim1 = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::confidenceFitLim1);
    _ballCfg.fitLim2 = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::confidenceFitLim2);
    _ballCfg.zLim1 = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::confidenceZLim1);
    _ballCfg.zLim2 = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::confidenceZLim2);
    _ballCfg.vLim1 = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::confidenceVLim1);
    _ballCfg.vLim2 = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::confidenceVLim2);
    _ballCfg.useOwnHighVision = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorBool::useOwnHighVision);
    _ballCfg.useFriendlyHighVision = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorBool::useFriendlyHighVision);
    _ballCfg.friendlyMeasurementsDistance = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::friendlyMeasurementsDistance);
    _ballCfg.blackListThresholdZ = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::blackListThresholdZ);
    _ballCfg.blackListFloatingDuration = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::blackListFloatingDuration);
    _ballCfg.blackListGroundDuration = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::blackListGroundDuration);
    _ballCfg.blackListDefault = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorBool::blackListDefault);

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
    return objA.getObjectMeasurement().timestamp < objB.getObjectMeasurement().timestamp;
}

bool checkDeltaAngles(const objectMeasurementCache &measurement, Vector3D const &expectedPos)
{
    // check if delta angles are small enough
    Position2D robotPos(measurement.getObjectMeasurement().cameraX, measurement.getObjectMeasurement().cameraY, measurement.getObjectMeasurement().cameraPhi);
    float camZ = measurement.getObjectMeasurement().cameraZ; 
    Vector3D measPos = measurement.getPositionFcs();
    // transform to robot perspective
    Position2D expectedFcs(expectedPos.x, expectedPos.y, 0.0);
    Position2D expectedRcs = Position2D(expectedFcs).transform_fcs2rcs(robotPos);
    Position2D measuredFcs(measPos.x, measPos.y, 0.0);
    Position2D measuredRcs = Position2D(measuredFcs).transform_fcs2rcs(robotPos);
    float expectedDistance = vectorsize(expectedRcs.xy());
    float measuredDistance = vectorsize(measuredRcs.xy());
    // prevent div by zero
    if (std::min(measuredDistance, expectedDistance) < 0.3)
    {   
        // if one of both is closeby, then we basically apply a proximity criterion: the other should also be closeby
        return std::max(measuredDistance, expectedDistance) < 0.3;
    }
    // angles
    float deltaAz = atan2(expectedRcs.x - robotPos.x, expectedRcs.y - robotPos.y) - atan2(measuredRcs.x - robotPos.x, measuredRcs.y - robotPos.y);
    float deltaEl = atan2(expectedPos.z - camZ, expectedDistance) - measurement.getObjectMeasurement().elevation;
    float threshold = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::solverTrackerConeTolerance);
    //tprintf("deltaAz=%.3f deltaEl=%.3f  %.3f %.3f %.3f %.3f", deltaAz, deltaEl, fabs(deltaAz), fabs(deltaEl), std::max(fabs(deltaAz), fabs(deltaEl)), threshold);
    return std::max(fabs(deltaAz), fabs(deltaEl)) < threshold;
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
        *  - calculate expected position by including speed vector
        *  - compare measurement with expected position, calculate angles from camera perspective
        *  - check angles (cone) against threshold; distance can hardly be trusted
        */

        Vector3D currentPos(_lastBallResult.getX(), _lastBallResult.getY(), _lastBallResult.getZ());
        float dt = double(measurement.getObjectMeasurement().timestamp) - double(_lastBallResult.getTimestamp());
        Vector3D expectedPos = currentPos + dt * Vector3D(_lastBallResult.getVX(), _lastBallResult.getVY(), _lastBallResult.getVZ());
        if (checkDeltaAngles(measurement, expectedPos))
        {
            // as far as discriminator is concerned, this measurement will now be processed
            // in principle it is added and to be used, but it may also be the case that we here already reject it
            // in which case the measurement is not added to the vector, but flag measurementIsAdded is set
            // TODO: make this nicer (rename measurementIsAdded to measurementIsProcessed? filter elsewhere?)
            bool accept = true;
            auto camType = measurement.getObjectMeasurement().source;
            bool isHighCam = (camType == cameraEnum::FRONTCAMERA) || (camType == cameraEnum::MULTICAMERA);
            if (isHighCam)
            {
                if (measurement.getObjectMeasurement().identifier.robotID == getRobotNumber())
                {
                    accept = _ballCfg.useOwnHighVision;
                }
                else
                {
                    accept = _ballCfg.useFriendlyHighVision;
                }
            }

            if (accept)
            {
                // invariant: measurements shall be sorted by time
                // optimization: only sort if needed, i.e. timestamp is not newer than the last one
                if (measurement.getObjectMeasurement().timestamp < _ballMeasurements.back().getObjectMeasurement().timestamp)
                {
                    _ballMeasurements.push_back(measurement);
                    std::sort(_ballMeasurements.begin(), _ballMeasurements.end(), sortOnIncreasingTime);
                }
                else
                {
                    _ballMeasurements.push_back(measurement);
                    // no sort required
                }
                // update time range
                _tmin = std::min(_tmin, measurement.getObjectMeasurement().timestamp);
                _tmax = std::max(_tmax, measurement.getObjectMeasurement().timestamp);

                // floating ball detection
                // see also isBlackListed()
                Vector3D pos = measurement.getPositionFcs();
                bool airborne = (pos.z > _ballCfg.blackListThresholdZ);
                rtime tMeas = measurement.getObjectMeasurement().timestamp;
                if (airborne)
                {
                    _lastAirTimeStamp = tMeas;
                    double floatingTime = tMeas - _lastGroundTimeStamp;
                    if (floatingTime > _ballCfg.blackListFloatingDuration)
                    {
                        // what magic is this?! floating -> blacklist (probably a lamp, T-shirt, or a poster, ...)
                        setBlackList(blackListType::BLACK);
                    }
                }
                else
                {
                    _lastGroundTimeStamp = tMeas;
                    double groundTime = tMeas - _lastAirTimeStamp;
                    if (groundTime > _ballCfg.blackListGroundDuration)
                    {
                        // this seems to be a real ball
                        setBlackList(blackListType::WHITE);
                    }
                }
                
                //auto m = measurement.getObjectMeasurement();
                //tprintf("added ball  id %5d  cam  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  pos  %9.5f  %9.5f  %9.5f",
                //    _trackerID, m.cameraX, m.cameraY, m.cameraZ, m.cameraPhi, m.azimuth, m.elevation, m.radius, m.confidence, pos.x, pos.y, pos.z); 

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

void ballTracker::setBlackList(blackListType b)
{
    _blackList = b;
    if (b == blackListType::BLACK)
    {
        TRACE("tracker id=%d is now blacklisted", _trackerID);
    }
    else if (b == blackListType::WHITE)
    {
        TRACE("tracker id=%d is now whitelisted", _trackerID);
    }
}

void ballTracker::calculateConfidence(rtime const t)
{

    // ported implementation from fbt_tracker_confidence.m
    // confidence is determined by calculating several sub-scores, each between 0.0 (bad) and 1.0 (good)
    // final confidence is the product of each sub-score

    // iterate once through all measurements, to extract some values
    _confDetails.withOmni = false;
    std::set<uint8_t> usedCams;
    for (auto it = _ballMeasurements.begin(); it != _ballMeasurements.end(); ++it)
    {
        usedCams.insert(it->getObjectMeasurement().identifier.robotID);
        if (it->getObjectMeasurement().source == cameraEnum::OMNIVISION)
        {
            _confDetails.withOmni = true;
        }
    }

    // out-of-bounds score
    float edgeTolerance = 0.5; // TODO reconfigurable
    float margin = 0.1;
    float boundX = margin + 0.5 * cEnvironmentField::getInstance().getWidth();
    float boundY = margin + 0.5 * cEnvironmentField::getInstance().getLength();
    float boundScoreX = 1.0 - std::min(std::max((fabs(_lastBallResult.getX()) - boundX) / edgeTolerance, 0.0), 1.0);
    float boundScoreY = 1.0 - std::min(std::max((fabs(_lastBallResult.getY()) - boundY) / edgeTolerance, 0.0), 1.0);
    _confDetails.boundScore = boundScoreX * boundScoreY;

    // camera score (more is better, omnivision beats frontvision)
    _confDetails.numCams = 0;
    _confDetails.numCams = usedCams.size();
    float camScale = _ballCfg.numCams * (1 + _ballCfg.omniPref);
    _confDetails.camScore = std::min(_confDetails.numCams, _ballCfg.numCams) * (1 + (_confDetails.withOmni && _ballCfg.omniPref)) / camScale;

    // number of measurements (more is better)
    _confDetails.measScore = std::min(_tracker.getNumGood(), _ballCfg.measLim) / (1.0 * _ballCfg.measLim);

    // age (higher is better) and freshness (time since last measurement - lower is better)
    _confDetails.ageScore = std::min(1.0, double(t - _t0) / _ballCfg.ageLim);
    _confDetails.freshScore = 1.0 - std::max(0.0, (double(t - _tmax) - _ballCfg.freshLim) / (_ballCfg.timeout - _ballCfg.freshLim));

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
        * _confDetails.fitScore * _confDetails.boundScore;

    // store
    _lastBallResult.setConfidence(confidence);
}

bool ballTracker::isBlackListed() const
{
    // there is a fundamental issue with detecting lob shots
    // suppose enemy is taking a free kick, where the receiving robot lobs immediately across the enemy which took the kick
    // TechUnited often does this 'tikkie takka'
    // then the goalie will see the ball first in the air and it needs to immediately respond to it (option A)
    // however, this will cause robots to also briefly respond to things like posters and lights, when they first are tracked
    // option B: more cauteous, only accept a ball after it has touched the ground
    // if friendly robots observe such a free kick from the side, this should still work out OK for the keeper

    // A. use this condition if we want aggressive but also glitchy behavior
    // any fresh flying tracker might be accepted, because it is not yet blacklisted
    if (!_ballCfg.blackListDefault)
    {
        return (_blackList == blackListType::BLACK);
    }

    // B. use this condition if we want cauteous behavior
    // any fresh flying tracker is by default rejected, until it touches the ground, at which point it is whitelisted
    return (_blackList != blackListType::WHITE);
}

void ballTracker::calculateBall(rtime const timestampNow, bool ownBallsFirst)
{
    try
    {
        // entry point of the algorithm
        std::vector<objectMeasurementCache> measurements = _ballMeasurements;
        // pre-filter?
        if (ownBallsFirst)
        {
            selectOwnOmniMeasurements(measurements);
        }
        _ownBallsFirst = ownBallsFirst; // for diagnostics
        // call solver, require at least one measurement present, otherwise we may not call solver
        if (measurements.size() == 0)
        {
            // return empty
            _lastBallResult = ballClass_t();
            _lastBallResult.setConfidence(0.0);
            _lastBallResult.setId(_trackerID);
            return;
        }
        // safe to call solver
        // TODO: get grouped measurements and mask of removed elements for tracing?
        _tracker.solve(measurements, timestampNow, true);
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

bool ballTracker::isTimedOut(rtime const t)
{
    try
    {
        bool retVal = false;

        /*
        * State when tracker is not longer valid
        * Current behavior: when list of measurements are empty
        * or when last measurement exceeds tracker timeout
        */
        cleanUpTimedOutBallMeasurements(t);
        retVal = _ballMeasurements.empty();
        if (!retVal)
        {
            auto timeout = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::solverTrackerTimeout);
            double time_diff = t - _tmax;
            if (time_diff > timeout)
            {
                //tprintf("BALL TIMEOUT delta=%.6f t=%s %.6f _tmax=%s %.6f", time_diff, t.toStr().c_str(), double(t), _tmax.toStr().c_str(), double(_tmax));
                retVal = true;
            }
        }

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

void ballTracker::selectOwnOmniMeasurements(std::vector<objectMeasurementCache> &measurements)
{
    try
    {
        int ownRobotId = getRobotNumber();
        for(auto i = measurements.begin(); i != measurements.end(); )
        {
            if ((i->getObjectMeasurement().identifier.robotID != ownRobotId)
                || (i->getObjectMeasurement().source != cameraEnum::OMNIVISION))
            {
                // Delete ball measurement
                i = measurements.erase(i);
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

void ballTracker::cleanUpTimedOutBallMeasurements(rtime const t)
{
    try
    {
        auto bufferSize = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::solverTrackerBuffer);

        // remove old measurements
        for (auto i = _ballMeasurements.begin(); i != _ballMeasurements.end(); )
        {
            double time_diff = _tmax - i->getObjectMeasurement().timestamp;
            if (time_diff > bufferSize)
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

void ballTracker::traceMeasurements() const
{
    for (auto imeas = _ballMeasurements.begin(); imeas != _ballMeasurements.end(); ++imeas)
    {
        objectMeasurement m = imeas->getObjectMeasurement();
        Vector3D ballPos = imeas->getPositionFcs();
        TRACE("measurement:  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f",
            m.cameraX, m.cameraY, m.cameraZ, m.cameraPhi,
            m.azimuth, m.elevation, m.radius, m.confidence,
            ballPos.x, ballPos.y, ballPos.z); 
    }
}

std::string ballTracker::toStr(rtime const tcurr, bool details)
{
    char buf[256];
    double age = tcurr - _t0;
    sprintf(buf, "%4d %4.2f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %5.1f %5d", 
            (int)_trackerID, _lastBallResult.getConfidence(), 
            _lastBallResult.getX(), _lastBallResult.getY(), _lastBallResult.getZ(), 
            _lastBallResult.getVX(), _lastBallResult.getVY(), _lastBallResult.getVZ(), 
            age, _tracker.getNumGood());
    std::string result = buf;
    if (details)
    {
        sprintf(buf, " %1d %1d %5d %4d %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f",
                (int)_blackList, _ownBallsFirst,
                _confDetails.numCams, _confDetails.withOmni, 
                _confDetails.camScore, _confDetails.ageScore, _confDetails.measScore, _confDetails.freshScore,
                _confDetails.zScore, _confDetails.vScore, _confDetails.boundScore,
                _confDetails.fitScore, _confDetails.fitQuality);
        result += buf;
    }
    return result;
}

std::string ballTracker::xyzDetailsStr()
{
    return _tracker.getDetailsStr();
}

