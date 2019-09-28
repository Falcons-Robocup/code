 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
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

#include "cDiagnostics.hpp"
#include "tracing.hpp"

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
}

ballDiscriminator::~ballDiscriminator()
/*
 *    Chuck Norris doesn't cheat death. He wins fair and square.
 */
{

}

void traceMeasurement(const objectMeasurementCache &measurementCache, int assignedTrackerId)
{
    ballMeasurement const &measurement = measurementCache.getObjectMeasurement();
    // trace raw measurements, for playback/analysis afterwards
    //TRACE("%2d  %d  %5d  %16.6f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f  %9.5f", 
    //    measurement.getID().robotID, assignedTrackerId, (int)measurement.getCameraType(), measurement.getTimestamp(),
    //    measurement.getCameraX(), measurement.getCameraY(), measurement.getCameraZ(), measurement.getCameraPhi(),
    //    measurement.getAzimuth(), measurement.getElevation(), measurement.getRadius(), measurement.getConfidence());
    // similar: performance tracing - we include xyz coordinates for easy plotting without having to perform calculations
    // Vector3D pos = measurementCache.getPositionFcs();
    //PTRACE("BM %2d %1d %16.6f  %9.6f %9.6f %9.6f  %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f  %.3f %d", 
    //       measurement.identifier.robotID, measurement.source, measurement.timestamp,
    //       pos.x, pos.y, pos.z, // perceived ball position
    //       // raw measurement details
    //       measurement.cameraX, measurement.cameraY, measurement.cameraZ, measurement.cameraPhi,
    //       measurement.azimuth, measurement.elevation, measurement.radius, measurement.confidence,
    //       assignedTrackerId); 
}        

void checkLatency(const ballMeasurement &measurement)
{
    // configurables
    float FUTURE_TOLERANCE = -0.05;
    float FUTURE_WARNING_TIMEOUT = 30.0;
    float LATENCY_TOLERANCE = 1e9; // 0.4;
    float LATENCY_WARNING_TIMEOUT = 15.0;
    // evaluate
    rtime t = rtime::now(); // TODO: move out of here ... bad for s(t)imulation and testing ...
    double delta = t - measurement.timestamp;
    if (delta < FUTURE_TOLERANCE)
    {
        TRACE_WARNING_TIMEOUT(FUTURE_WARNING_TIMEOUT, "future ball data received from r%d (dt=%.3fs)", measurement.identifier.robotID, -delta);
    }
    if (delta > LATENCY_TOLERANCE)
    {
        TRACE_WARNING_TIMEOUT(LATENCY_WARNING_TIMEOUT, "high-latency data received from r%d (dt=%.3fs)", measurement.identifier.robotID, delta);
    }
}

bool ignoreHighVision(const ballMeasurement &measurement)
{
    // rather duplicate with code in ballTracker.cpp ...
    // frontVision is going to be removed anyway in 2018
    bool isFrontCam = (measurement.source == cameraEnum::FRONTCAMERA);
    bool isMultiCam = (measurement.source == cameraEnum::MULTICAMERA);
    bool isHighVision = isFrontCam || isMultiCam;
    if (!isHighVision)
    {
        // no need to ignore
        return false;
    }
    // HACK 20180308: ignore frontCam balls within omniVision range, to prevent swaffelen
    if ((measurement.radius < 3.0) && isFrontCam)
    {
        return true;
    }
    if (measurement.identifier.robotID == getRobotNumber())
    {
        return !ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorBool::useOwnHighVision);
    }
    return !ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorBool::useFriendlyHighVision);
}

void ballDiscriminator::addMeasurement(const ballMeasurement &measurement)
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

        if (ignoreHighVision(measurement))
        {
            // reject because we do not want to use this frontVision measurement
            traceMeasurement(bm, 0);
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
                TRACE("added to existing tracker %d at (%6.2f, %6.2f, %6.2f)", ball.getId(), ball.getX(), ball.getY(), ball.getZ());
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
            TRACE("creating new tracker %d at (%6.2f, %6.2f, %6.2f)", ball.getId(), ball.getX(), ball.getY(), ball.getZ());
            assignedTrackerId = ball.getId();
        }

        // trace raw measurements, for playback/analysis afterwards
        traceMeasurement(bm, (int)assignedTrackerId);
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

void ballDiscriminator::getMeasurementsToSync(std::vector<ballMeasurement> &measurements)
{
    if (_balls.size())
    {
        // only sync from best ball - this works magnetically: best tracker on other robots gets stronger, all others starve
        for(auto it = _ballTrackers.begin(); it != _ballTrackers.end(); it++)
        {
            if (it->getBall().getId() == _balls[0].getId())
            {
                // select 3 newest, to achieve a bit of data redundancy
                std::vector<objectMeasurementCache> ms = it->getBallMeasurements();
                for (auto it2 = ms.rbegin(); (it2 != ms.rend()) && (measurements.size() < 3); ++it2)
                {
                    auto m = it2->getObjectMeasurement();
                    m.confidence = it->getBall().getConfidence();
                    measurements.push_back(m);
                }
            }
        }
    }
    else
    {
        // take single best one from all non-blacklisted trackers, to enable robots to combine data and make the best ball emerge
        for (auto it = _ballTrackers.begin(); it != _ballTrackers.end(); it++)
        {
            if (!it->isBlackListed())
            {
                std::vector<objectMeasurementCache> ms = it->getBallMeasurements();
                if (ms.size())
                {
                    auto m = ms.back().getObjectMeasurement();
                    m.confidence = it->getBall().getConfidence();
                    measurements.push_back(m);
                }
            }
        }
    }
    // do not share high measurements (frontVision, multiCam) if so configured
    if (!ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorBool::shareHighVision))
    {
        for (auto it = measurements.begin(); it != measurements.end(); )
        {
            if ((it->source == cameraEnum::FRONTCAMERA) || (it->source == cameraEnum::MULTICAMERA))
            {
                it = measurements.erase(it);
            }
            else
            {
                it++;
            }
        }
    }
}

void ballDiscriminator::traceTrackers(rtime const tcurr, bool all)
{
    // for each good ball, show a line with details
    bool tracedBr = false;
    for(auto it = _ballTrackers.begin(); it != _ballTrackers.end(); it++)
    {
        // get tracker details
        std::string trackerInfo = it->toStr(tcurr, true);
        // TODO what was the zero intended for?
        std::string tag = "BT"; // ball tracker details (compare confidence heuristics)
        if (_balls.size() && (it->getBall().getId() == _balls[0].getId()))
        {
            tag = "BR"; // ball result
            tracedBr = true;
        }
        //tprintf("%2s %16.6f %3d %2d %s", tag.c_str(), double(tcurr), _ballTrackers.size(), 0, trackerInfo.c_str());
    }
    if (!tracedBr)
    {
        //tprintf("BR %16.6f noBall", double(tcurr));
    }
    // additionally, show tracker internal details, but only for the best ball, because this is quite data-heavy
    if (_balls.size())
    {
        for(auto it = _ballTrackers.begin(); it != _ballTrackers.end(); it++)
        {
            if (it->getBall().getId() == _balls[0].getId())
            {
                //std::string details = it->xyzDetailsStr();
                //tprintf("BD %16.6f %s", tcurr, details.c_str());
            }
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

void ballDiscriminator::selectGoodBalls(rtime const timeNow)
{
    // reset
    _balls.clear();

    // only get 'good' balls, i.e. confidence must be high enough, not blacklisted
    float thresholdGood = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::confidenceGoodLimit);
    for(auto it = _ballTrackers.begin(); it != _ballTrackers.end(); it++)
    {
        ballClass_t ball = it->getBall();
        bool good = (ball.getConfidence() > thresholdGood) && !it->isBlackListed();
        if (good)
        {
            ball.setIsValid(true);
            _balls.push_back(ball);
        }
    }

    // sort on confidence
    std::sort(_balls.begin(), _balls.end());
    int numGood = _balls.size();

    // count, warn?
    auto threshold = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorIntegers::numberOfBallsWarningThreshold);
    if ((int)_balls.size() > threshold)
    {
        TRACE_INFO_TIMEOUT(3, "number of balls detected: %d", _balls.size());
    }
        
    // if there are no good balls, then we can flag a few maybe balls as (barely) good enough
    if (numGood == 0)
    {
        TRACE("no good balls detected, going into confidence fallback");
        // select candidates
        auto thresholdMaybe = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::confidenceMaybeLimit);
        for (auto it = _ballTrackers.begin(); it != _ballTrackers.end(); it++)
        {
            ballClass_t ball = it->getBall();
            bool good = (ball.getConfidence() > thresholdMaybe) && !it->isBlackListed();
            if (good)
            {
                ball.setIsValid(true);
                _balls.push_back(ball);
            }
        }
        // sort on decreasing confidence
        std::sort(_balls.begin(), _balls.end());
        int maxMaybeBalls = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorIntegers::maxMaybeBalls);
        // select best N balls
        if ((int)_balls.size() > maxMaybeBalls)
        {
            _balls.resize(maxMaybeBalls);
        }
        TRACE("promoted %d maybe-balls to good", (int)_balls.size());
    }
}

void ballDiscriminator::performCalculation(rtime const timeNow, Vector2D const &pos)
{
    TRACE_FUNCTION("");
    try
    {
        //TRACE("#trackers=%d", (int)_ballTrackers.size());
        //TRACE("cleaning up (t=%16.6f)", timeNow);
        removeTimedOutTrackers(timeNow);
        //TRACE("remaining #trackers=%d", (int)_ballTrackers.size());

        // clear result
        _balls.clear();
        
        // ownBallsFirst
        // in case ball is close by, then we choose to fully rely on own measurements, 
        // because friendly measurements tend to cause distortions
        ownBallsFirst(timeNow, pos);
        //TRACE("#balls=%d", (int)_balls.size());
        
        // no ball closeby? then use all data
        if (_balls.size() == 0)
        {
            for(auto it = _ballTrackers.begin(); it != _ballTrackers.end(); it++)
            {
                it->calculateBall(timeNow);
            }
            selectGoodBalls(timeNow);
        }
        //TRACE("#balls=%d", (int)_balls.size());

        traceTrackers(timeNow, true); // for now trace ALL
        //TRACE("#balls=%d", (int)_balls.size());
        
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

void ballDiscriminator::ownBallsFirst(rtime const timeNow, Vector2D const &pos)
{
    try
    {
        // select closest ball (tracker)
        float ownBallsFirstDistance = ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::friendlyMeasurementsDistance);
        float bestConfidence = 0.0;

        ballTracker *bestTracker = NULL;
        for (auto it = _ballTrackers.begin(); it != _ballTrackers.end(); it++)
        {
            // check if closeby
            ballClass_t ball = it->getBall();
            Vector2D ballPos(ball.getX(), ball.getY());
            float distance = (ballPos - pos).size();
            if (!it->isBlackListed() && (distance < ownBallsFirstDistance))
            {
                // recalculate with own measurements only
                it->calculateBall(timeNow, true);
                ball = it->getBall();
                ballPos = Vector2D(ball.getX(), ball.getY());
                float confidence = ball.getConfidence();
                if (confidence > bestConfidence)
                {
                    bestTracker = &(*it);
                    bestConfidence = confidence;
                }
            }
        }

        if (bestTracker != NULL)
        {
            TRACE("ownBallsFirst for tracker %d", bestTracker->getBall().getId());
            // overrule resulting _balls vector
            _balls.clear();
            _balls.push_back(bestTracker->getBall());
            _balls.back().setIsValid(true);
        }
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}


std::vector<ballClass_t> ballDiscriminator::getBalls() const
{
    try
    {
        //TRACE("#balls=%d", (int)_balls.size());
        return _balls;
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

void ballDiscriminator::removeTimedOutTrackers(rtime const timeNow)
{
    try
    {
        for(auto it = _ballTrackers.begin(); it != _ballTrackers.end(); )
        {
            if(it->isTimedOut(timeNow))
            {
                //TRACE("deleting ballTracker(id=%d)", it->getBall().getId());
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

void ballDiscriminator::fillDiagnostics(diagWorldModel &diagnostics)
{
    // note: 
    // * resulting balls and their confidence should be logged (shared) via worldState
    // * ball tracker details (confidence breakdown, measurement outlier removal, etc. etc.) should be logged (local) via ballTracking
    diagnostics.shared.numBallTrackers = _ballTrackers.size();
    diagnostics.shared.bestTrackerId = -1;
    diagnostics.shared.ownBallsFirst = false;
    
    // diagBallTracker  ball;
    
    if (_balls.size())
    {
        diagnostics.shared.bestTrackerId = _balls[0].getId();
        for (auto it = _ballTrackers.begin(); it != _ballTrackers.end(); ++it)
        {
            if ((int)it->getBall().getId() == diagnostics.shared.bestTrackerId)
            {
                diagnostics.shared.ownBallsFirst = it->getOwnBallsFirst();
                /*
                ball.id = diagnostics.shared.bestTrackerId;
                
                
                for (auto it2 = it->getBallMeasurements().begin(); it2 != it->getBallMeasurements().end(); ++it2)
                {
                    ball.positionFcs.push_back(it2->getPositionFcs());
                    ball.timestamps.push_back(it2->getObjectMeasurement().timestamp);
                    // TODO: fill results?
                } 
                
                // TODO: fill other balls?
                diagnostics.local.balls.push_back(ball); */
            }
        }
    }
}

