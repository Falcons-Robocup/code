// Copyright 2016-2022 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ballAdministrator.cpp
 *
 *  Created on: Sep 6, 2016
 *      Author: Tim Kouters
 */

#include "int/administrators/ballAdministrator.hpp"

#include <algorithm>

#include "int/administrators/ballDiscriminator.hpp"
#include "int/administrators/gaussian3DBallDiscriminator.hpp"

#include "cDiagnostics.hpp"
#include "tracing.hpp"

ballAdministrator::ballAdministrator(WorldModelConfig& wmConfig)
    : _wmConfig(wmConfig)
/*!
 * \brief Administrates ball measurements
 *
 * It has 2 purposes:
 *  - Separating local and remote measurements for syncing
 *  - Making sure measurements are only fed once to the discriminator
 *
 */
{
    _ownRobotID = getRobotNumber();
    _ballMeasurements.clear();
    _overruledBalls.clear();

    int ballTrackerAlgorithm = wmConfig.getConfiguration().ballTracker.ballTrackerAlgorithm;
    if(ballTrackerAlgorithm == 0)
    {        
        _ballDiscriminator = new ballDiscriminator(wmConfig);
    }
    else if(ballTrackerAlgorithm == 1)
    {
        _ballDiscriminator = new Gaussian3DBallDiscriminator(&wmConfig);
    }
    else
    {
        TRACE_WARNING("Invalid ball tracker algorithm %d, using default option.", ballTrackerAlgorithm);
        _ballDiscriminator = new ballDiscriminator(wmConfig);
    }
}

ballAdministrator::~ballAdministrator()
/*
 * Chuck Norris' calendar goes straight from March 31st to April 2nd.
 * No one fools Chuck Norris.
 */
{
    delete _ballDiscriminator;
}

void ballAdministrator::appendBallMeasurements(const std::vector<ballMeasurement> measurements)
{
    //TRACE("> #meas=%d", (int)measurements.size());
    try
    {
        for(auto itMeasurement = measurements.begin(); itMeasurement != measurements.end(); itMeasurement++)
        {
            auto it = _ballMeasurements.find(itMeasurement->identifier);

            // If not found, add it and feed to object discriminator
            if(it == _ballMeasurements.end())
            {
            	_ballMeasurements[itMeasurement->identifier] = (*itMeasurement);
            	_ballDiscriminator->addMeasurement(*itMeasurement);
            }
        }
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
    //TRACE("<");
}

void ballAdministrator::appendBallPossessionMeasurements(const Vector3D& ball_pos, uint8_t robotID, rtime timestamp)
{
    _ballDiscriminator->addPossessionMeasurement(ball_pos, robotID, timestamp);
}

void ballAdministrator::overruleBall(const ballClass_t ball)
{
    try
    {
        _overruledBalls[ball.getId()] = ball;
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

void ballAdministrator::getLocalBallMeasurements(std::vector<ballMeasurement> &measurements)
{
    // this function is called by wmSyncAdapter, to get the data to be sent to friendly robots
    //TRACE(">");
    try
    {
        measurements.clear();
        
        // note: wmSync buffer is rather small
        // so select only measurements which are not attributed to blacklisted trackers; also sort on decreasing tracker confidence
        _ballDiscriminator->getMeasurementsToSync(measurements);

    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
    //TRACE("< #meas=%d", (int)measurements.size());
}

void ballAdministrator::performCalculation(rtime const timeNow, Vector2D const &pos)
{
    std::ostringstream ss;
    ss << "pos: (" << pos.x << ", " << pos.y << ")";
    TRACE_FUNCTION(ss.str().c_str());
    try
    {
        // in case of simulation, _overruledBalls is used instead of _ballDiscriminator
        if (_overruledBalls.size() == 0)
        {
            cleanUpTimedOutBallMeasurements(timeNow);
            _ballDiscriminator->performCalculation(timeNow, pos);
        }
        else
        {
            // send the simulated ball(s) to visualizer
            std::vector<ballClass_t> balls;
            for(auto it = _overruledBalls.begin(); it != _overruledBalls.end(); it++)
            {
            	balls.push_back(it->second);
            }
        }
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
    //TRACE("<");
}

void ballAdministrator::getBalls(std::vector<ballClass_t> &balls)
{
    //TRACE(">");
    try
    {
        if(_overruledBalls.empty())
        {
            balls = _ballDiscriminator->getBalls();
        }
        else
        {
            for(auto it = _overruledBalls.begin(); it != _overruledBalls.end(); it++)
            {
            	balls.push_back(it->second);
            }
        }
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
    TRACE("< numballs=%d", (int)balls.size());
}

void ballAdministrator::cleanUpTimedOutBallMeasurements(rtime const timeNow)
{
    TRACE_FUNCTION("");
    size_t origSize = _ballMeasurements.size();
    try
    {
        double maxTimeToLive = _wmConfig.getConfiguration().administration.ball_timeout;

        for(auto i = _ballMeasurements.begin(); i != _ballMeasurements.end(); )
        {
            double time_diff = timeNow - i->second.timestamp;
            if(time_diff > maxTimeToLive)
            {
                // Delete ball measurement
            	i = _ballMeasurements.erase(i);
            }
            else
            {
            	i++;
            }
        }

        for(auto i = _overruledBalls.begin(); i != _overruledBalls.end(); )
        {
            double time_diff = timeNow - i->second.getTimestamp();
            if(time_diff > maxTimeToLive)
            {
                // Delete robot with iterator
            	i = _overruledBalls.erase(i);
            }
            else
            {
            	i++;
            }
        }

    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
    std::ostringstream ss;
    ss << "origSize: " << origSize << "; newSize: " << _ballMeasurements.size();
    TRACE(ss.str().c_str());
}

void ballAdministrator::fillDiagnostics(diagWorldModel &diagnostics)
{
    _ballDiscriminator->fillDiagnostics(diagnostics);
}

