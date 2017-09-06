 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ballDiscriminator.cpp
 *
 *  Created on: Sep 6, 2016
 *      Author: Tim Kouters
 */

#include "int/administrators/ballDiscriminator.hpp"

#include "int/configurators/ballTrackerConfigurator.hpp"

#include <algorithm>

#include "cDiagnosticsEvents.hpp"

ballDiscriminator::ballDiscriminator()
/*!
 * \brief Administrates ball trackers
 *
 * Each tracker represents one ball
 * Hence multiple trackers mean multiple balls are detected and tracked
 *
 */
{
	_ballTrackers.clear();
	_diagSender = NULL;
}

ballDiscriminator::~ballDiscriminator()
/*
 *	Chuck Norris doesn't cheat death. He wins fair and square.
 */
{

}

void traceMeasurement(const ballMeasurementType &measurement, int assignedTrackerId)
{
    // trace raw measurements, for playback/analysis afterwards
    TRACE("%2d  %d  %5d  %16.6f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f", 
        measurement.getID().robotID, assignedTrackerId, (int)measurement.getCameraType(), measurement.getTimestamp(),
        measurement.getCameraX(), measurement.getCameraY(), measurement.getCameraZ(), measurement.getCameraPhi(),
        measurement.getAzimuth(), measurement.getElevation(), measurement.getRadius(), measurement.getConfidence());
}		

void checkLatency(const ballMeasurementType &measurement)
{
    // configurables
    float FUTURE_TOLERANCE = -0.05;
    float FUTURE_WARNING_TIMEOUT = 30.0;
    float LATENCY_TOLERANCE = 0.4;
    float LATENCY_WARNING_TIMEOUT = 15.0;
    // evaluate
    double t = getTimeNow();
    double delta = t - measurement.getTimestamp();
    if (delta < FUTURE_TOLERANCE)
    {
        TRACE_WARNING_TIMEOUT(FUTURE_WARNING_TIMEOUT, "future ball data received from r%d (dt=%.3fs)", measurement.getID().robotID, -delta);
    }
    if (delta > LATENCY_TOLERANCE)
    {
        TRACE_WARNING_TIMEOUT(LATENCY_WARNING_TIMEOUT, "high-latency data received from r%d (dt=%.3fs)", measurement.getID().robotID, delta);
    }
}

void ballDiscriminator::addMeasurement(const ballMeasurementType &measurement)
{
    try
    {
        // wrap the measurement, calculate ball (x,y,z) in FCS
        objectMeasurementCache bm(measurement);

        // check if measurement should be rejected 
        Vector3D ballPos = bm.getPositionFcs();
        if (ballPos.z > MAX_BALL_HEIGHT)
        {
            TRACE("rejected ball out of bounds (z=%6.2f)", ballPos.z);
            return;
        }

        bool useFrontVision = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorBool::useFrontVision);
        bool isFrontVision = (measurement.getCameraType() == cameraType::FRONTCAMERA);
        if (isFrontVision && !useFrontVision)
        {
            // reject because we do not want to use this frontVision measurement
            traceMeasurement(measurement, 0);
            return;
        }
	    
        // latency check (e.g. wifi and wmSync might cause delay)
        checkLatency(measurement);
	    
        bool measurementInserted = false;
        size_t assignedTrackerId = 0;

        for(auto it = _ballTrackers.begin(); ((it != _ballTrackers.end()) && (!measurementInserted)); it++)
        {
            it->addBallMeasurement(bm, measurementInserted);
            if (measurementInserted)
            {
                ballClass_t ball = it->getBall();
                TRACE("added to existing tracker %d at (%6.2f, %6.2f)", ball.getId(), ball.getX(), ball.getY());
                assignedTrackerId = ball.getId();
            }
        }

        /*
        * Ball measurement rejected by all trackers
        * Create new trackers and add to vector
        */
        if(!measurementInserted)
        {
            ballTracker newTracker(measurement);
            _ballTrackers.push_back(newTracker);
            ballClass_t ball = newTracker.getBall();
            TRACE("creating new tracker %d at (%6.2f, %6.2f)", ball.getId(), ball.getX(), ball.getY());
            assignedTrackerId = ball.getId();
        }

        // trace raw measurements, for playback/analysis afterwards
        traceMeasurement(measurement, (int)assignedTrackerId);
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

void ballDiscriminator::traceTrackers(const double tcurr, bool all)
{
    // count the number of good balls
    int num_good = 0;
	for(auto it = _ballTrackers.begin(); it != _ballTrackers.end(); it++)
	{
	    num_good += it->good();
    }
    
    // for each good ball, show a line with details
	for(auto it = _ballTrackers.begin(); it != _ballTrackers.end(); it++)
	{
	    if (it->good() || all)
	    {
	        // get tracker details
	        std::string trackerInfo = it->toStr(tcurr, true);
	        TRACE("%16.6f %3d %2d %s", tcurr, _ballTrackers.size(), num_good, trackerInfo.c_str());
        }
    }
}

// template found on http://stackoverflow.com/a/12399290
template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> &v) 
{
    // initialize original index locations
    std::vector<size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);

    // sort indexes based on comparing values in v
    std::sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] > v[i2];});

    return idx;
}

void ballDiscriminator::calculateConfidence(const double timeNow)
{
    // calculate per-tracker confidence 
    std::vector<float> confList;
    auto thresholdGood = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::confidenceGoodLimit);
    int numGood = 0;
	for(auto it = _ballTrackers.begin(); it != _ballTrackers.end(); it++)
	{
    	ballClass_t ball = it->getBall();
	    confList.push_back(ball.getConfidence());
	    numGood += it->good();
    }
    TRACE("#confList=%d #good=%d", (int)confList.size(), numGood);
    if (confList.size() == 0)
    {   
        // done
        return;
    }
    // note: calculateConfidence will flag good balls, but if there are none, 
    // then we can flag a few maybe balls as (barely) good enough
    float maxConfidence = *std::max_element(confList.begin(), confList.end());
    auto thresholdMaybe = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::confidenceMaybeLimit);
    if ((maxConfidence < thresholdGood) && (numGood == 0))
    {
        TRACE("no good balls detected, going into confidence fallback");
        int maxMaybeBalls = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorIntegers::maxMaybeBalls);
        // select N best balls
        int count = 0;
        for (auto i: sort_indexes(confList)) 
        {
            if ((confList[i] > thresholdMaybe) && (count < maxMaybeBalls))
            {
                _ballTrackers[i].setGood();
                count++;
            }
        }
        TRACE("promoted %d maybe-balls to good", count);
    }
}

void ballDiscriminator::performCalculation(const double timeNow)
{
	try
	{
	    TRACE("#trackers=%d", (int)_ballTrackers.size());
	    TRACE("cleaning up (t=%16.6f)", timeNow);
		removeTimedOutTrackers(timeNow);
	    TRACE("remaining #trackers=%d", (int)_ballTrackers.size());
		for(auto it = _ballTrackers.begin(); it != _ballTrackers.end(); it++)
		{
			it->calculateBall(timeNow);
		}
		calculateConfidence(timeNow);
		
		traceTrackers(timeNow, true); // for now trace ALL
		
		sendDiagnostics();
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void ballDiscriminator::enableDiagnostics()
{
    _diagSender = new diagnostics::cDiagnosticsSender<rosMsgs::t_diag_wm_ball>(diagnostics::DIAG_WM_BALL, 10, false);
}

void ballDiscriminator::sendSimDiagnostics(std::vector<ballClass_t> const &balls)
{
    if (_diagSender)
    {
		rosMsgs::t_diag_wm_ball diagMsg;
		for(auto it = balls.begin(); it != balls.end(); it++)
		{
            rosMsgs::t_ball ballMsg;
            ballClass_t ball = *it;
            ballMsg.id = ball.getId();
            ballMsg.x = ball.getX();
            ballMsg.y = ball.getY();
            ballMsg.z = ball.getZ();
            ballMsg.vx = ball.getVX();
            ballMsg.vy = ball.getVY();
            ballMsg.vz = ball.getVZ();
            ballMsg.confidence = 1.0; // simulated balls have perfect confidence score
            diagMsg.balls.push_back(ballMsg);
        }
        _diagSender->set(diagMsg);
    }
}

void ballDiscriminator::sendDiagnostics()
{
    if (_diagSender)
    {
		rosMsgs::t_diag_wm_ball diagMsg;
		diagMsg.numTrackers = (int)_ballTrackers.size();
		for(auto it = _ballTrackers.begin(); it != _ballTrackers.end(); it++)
		{
            if (it->good())
            {
                rosMsgs::t_ball ballMsg;
                ballClass_t ball = it->getBall();
                ballMsg.id = ball.getId();
                ballMsg.x = ball.getX();
                ballMsg.y = ball.getY();
                ballMsg.z = ball.getZ();
                ballMsg.vx = ball.getVX();
                ballMsg.vy = ball.getVY();
                ballMsg.vz = ball.getVZ();
                ballMsg.confidence = ball.getConfidence();
                diagMsg.balls.push_back(ballMsg);
                // confidence details, useful for debugging and tuning in visualizer
                confidenceDetails confDetails = it->getDetails();
                rosMsgs::t_ball_details ballDetailMsg;
                ballDetailMsg.camScore = confDetails.camScore;
                ballDetailMsg.measScore = confDetails.measScore;
                ballDetailMsg.ageScore = confDetails.ageScore;
                ballDetailMsg.freshScore = confDetails.freshScore;
                ballDetailMsg.vScore = confDetails.vScore;
                ballDetailMsg.zScore = confDetails.zScore;
                ballDetailMsg.fitScore = confDetails.fitScore;
                ballDetailMsg.fitQuality = confDetails.fitQuality;
                diagMsg.details.push_back(ballDetailMsg);
            }
        }
        _diagSender->set(diagMsg);
    }
}

std::vector<ballClass_t> ballDiscriminator::getBalls() const
{
	try
	{
		std::vector<ballClass_t> balls;

        // only get 'good' balls, i.e. confidence must be high enough
		for(auto it = _ballTrackers.begin(); it != _ballTrackers.end(); it++)
		{
            if (it->good())
            {
    			balls.push_back(it->getBall());
			}
		}
		
		// Sort on confidence
		std::sort(balls.begin(), balls.end());

		// TODO update/check blacklist

        auto threshold = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorIntegers::numberOfBallsWarningThreshold);
		if((int)balls.size() > threshold)
		{
			TRACE_INFO("Number of balls detected: %d", balls.size());
		}
		
		return balls;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void ballDiscriminator::removeTimedOutTrackers(const double timeNow)
{
	try
	{
		for(auto it = _ballTrackers.begin(); it != _ballTrackers.end(); )
		{
			if(it->isTimedOut(timeNow))
			{
			    TRACE("deleting ballTracker(id=%d)", it->getBall().getId());
				it = _ballTrackers.erase(it);
			}
			else
			{
			    it++;
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
