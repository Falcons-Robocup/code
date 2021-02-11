// Copyright 2016-2020 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ballDiscriminator.cpp
 *
 *  Created on: Sep 6, 2016
 *      Author: Tim Kouters
 */

#include "int/administrators/ballDiscriminator.hpp"

#include <algorithm>
#include <numeric>

#include "cDiagnostics.hpp"
#include "tracing.hpp"

ballDiscriminator::ballDiscriminator(const WorldModelConfig& wmConfig)
    : _wmConfig(wmConfig)
/*!
 * \brief Administrates ball trackers
 *
 * Each tracker represents one ball
 * Hence multiple trackers mean multiple balls are detected and tracked
 *
 */
{
    _ownRobotId = getRobotNumber();
    _ballTrackers.clear();
}

ballDiscriminator::~ballDiscriminator()
/*
 *    Chuck Norris doesn't cheat death. He wins fair and square.
 */
{

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

bool ballDiscriminator::ignoreHighVision(const ballMeasurement &measurement)
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
    if (measurement.identifier.robotID == _ownRobotId)
    {
        return !_wmConfig.getConfiguration().ballTracker.useOwnHighVision;
    }
    return !_wmConfig.getConfiguration().ballTracker.useFriendlyHighVision;
}

void ballDiscriminator::addMeasurement(const ballMeasurement &measurement)
{
    TRACE_FUNCTION("");
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
            return;
        }

        // latency check (e.g. wifi and wmSync might cause delay)
        checkLatency(measurement);

        bool measurementInserted = false;

        for(auto it = _ballTrackers.begin(); ((it != _ballTrackers.end()) && (!measurementInserted)); it++)
        {
            it->addBallMeasurement(bm, measurementInserted);
            if (measurementInserted)
            {
                ballClass_t ball = it->getBall();
                TRACE("added to existing tracker %d at (%6.2f, %6.2f, %6.2f)", (int)ball.getId(), ball.getX(), ball.getY(), ball.getZ());
            }
        }

        /*
        * Ball measurement rejected by all trackers
        * Create new trackers and add to vector
        */
        if(!measurementInserted)
        {
            ballTracker newTracker(measurement, &_wmConfig);
            _ballTrackers.push_back(newTracker);
            ballClass_t ball = newTracker.getBall();
            TRACE("creating new tracker %d at (%6.2f, %6.2f, %6.2f)", (int)ball.getId(), ball.getX(), ball.getY(), ball.getZ());
        }
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
    if (!_wmConfig.getConfiguration().ballTracker.shareHighVision)
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

void ballDiscriminator::traceTrackers(rtime tcurr, bool all)
{
    TRACE_FUNCTION("");
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
        TRACE("%2s %16.6f %3d %2d %s", tag.c_str(), double(tcurr), (int)_ballTrackers.size(), 0, trackerInfo.c_str());
    }
    if (!tracedBr)
    {
        TRACE("BR %16.6f noBall", double(tcurr));
    }
    // additionally, show tracker internal details, but only for the best ball, because this is quite data-heavy
    if (_balls.size())
    {
        for(auto it = _ballTrackers.begin(); it != _ballTrackers.end(); it++)
        {
            if (it->getBall().getId() == _balls[0].getId())
            {
                std::string details = it->xyzDetailsStr();
                TRACE("BD %16.6f %s", tcurr, details.c_str());
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

void ballDiscriminator::selectGoodBalls(rtime timeNow)
{
    TRACE_FUNCTION("");
    // reset
    _balls.clear();

    // only get 'good' balls, i.e. confidence must be high enough, not blacklisted
    float thresholdGood = _wmConfig.getConfiguration().ballTracker.confidenceGoodLimit;
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
    auto threshold = _wmConfig.getConfiguration().ballTracker.numberOfBallsWarningThreshold;
    if ((int)_balls.size() > threshold)
    {
        TRACE_INFO_TIMEOUT(3, "number of balls detected: %d", _balls.size());
    }

    // if there are no good balls, then we can flag a few maybe balls as (barely) good enough
    if (numGood == 0)
    {
        TRACE("no good balls detected, going into confidence fallback");
        // select candidates
        auto thresholdMaybe = _wmConfig.getConfiguration().ballTracker.confidenceMaybeLimit;
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
        int maxMaybeBalls = _wmConfig.getConfiguration().ballTracker.maxMaybeBalls;
        // select best N balls
        if ((int)_balls.size() > maxMaybeBalls)
        {
            _balls.resize(maxMaybeBalls);
        }
        TRACE("promoted %d maybe-balls to good", (int)_balls.size());
    }
}

void ballDiscriminator::performCalculation(rtime timeNow, Vector2D const &pos)
{
    TRACE_FUNCTION("");
    try
    {
        TRACE("#trackers=%d", (int)_ballTrackers.size());
        TRACE("cleaning up (t=%16.6f)", timeNow);
        removeTimedOutTrackers(timeNow);
        TRACE("remaining #trackers=%d", (int)_ballTrackers.size());

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
            TRACE("calculate all balls");
            for(auto it = _ballTrackers.begin(); it != _ballTrackers.end(); it++)
            {
                it->calculateBall(timeNow);
            }
            selectGoodBalls(timeNow);
        }
        else
        {
            TRACE("not calculating all balls");
        }
        //TRACE("#balls=%d", (int)_balls.size());

        //traceTrackers(timeNow, true); // for now trace ALL
        //TRACE("#balls=%d", (int)_balls.size());

    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

void ballDiscriminator::ownBallsFirst(rtime timeNow, Vector2D const &pos)
{
    TRACE_FUNCTION("");
    try
    {
        // select closest ball (tracker)
        float ownBallsFirstDistance = _wmConfig.getConfiguration().ballTracker.friendlyMeasurementsDistance;
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

void ballDiscriminator::removeTimedOutTrackers(rtime timeNow)
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
    TRACE_FUNCTION("");
    // note:
    // * resulting balls and their confidence should be logged (shared) via worldState
    // * ball tracker details (confidence breakdown, measurement outlier removal, etc. etc.) should be logged (local) via ballTracking
    diagnostics.shared.numBallTrackers = _ballTrackers.size();
    diagnostics.shared.bestTrackerId = -1;
    diagnostics.shared.ownBallsFirst = false;
    diagnostics.local.timestamp = diagnostics.shared.timestamp;
    diagnostics.local.balls.clear();
    bool addAllDetails = true; // enabling this enables extra plotting capabilities in plot_balls.py,
    // but at the cost of increasing RDL size quite a bit
    diagnostics.shared.bestTrackerId = -1;
    if (_balls.size())
    {
        diagnostics.shared.bestTrackerId = _balls[0].getId();
    }
    for (auto it = _ballTrackers.begin(); it != _ballTrackers.end(); ++it)
    {
        bool isBestBall = ((int)it->getBall().getId() == diagnostics.shared.bestTrackerId);
        if (isBestBall)
        {
            diagnostics.shared.ownBallsFirst = it->getOwnBallsFirst();
        }
        if (isBestBall || addAllDetails)
        {
            diagBallTracker btdiag;
            it->makeDiagnostics(btdiag, diagnostics.shared.timestamp);
            diagnostics.local.balls.push_back(btdiag);
        }
    }
}

