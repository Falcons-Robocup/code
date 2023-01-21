// Copyright 2019-2022 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * simworld.cpp
 *
 *  Created on: Feb 2, 2019
 *      Author: Coen Tempelaars
 */

#include "int/simworld.hpp"

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>

#include "int/gameDataFactory.hpp"
#include "int/robotCapabilities.hpp"
#include "int/RTDBconfigAdapter.hpp"
#include "int/RTDBgameDataAdapter.hpp"
#include "int/RTDBmotionAdapter.hpp"
#include "int/RTDBtimeAdapter.hpp"
#include "int/simulationCapabilities.hpp"
#include "int/simworldGameDataFactory.hpp"
#include "tracing.hpp"
#include "ftime.hpp"

#include <mutex>
#include <condition_variable>

class Simworld::HeartbeatWaiter
{
public:
    HeartbeatWaiter(TeamID team, RobotID robot, const AbstractTimeAdapter& timeAdapter)
    {
        auto waitLoop = [this, team, robot, &timeAdapter]
            {
                while (true)
                {
                    // wait for a signal to start observing the heartbeat
                    std::unique_lock<std::mutex> lock(_mutex);
                    _conditionVar.wait(lock, [this]{ return isRunning(); });
                    timeAdapter.waitForPutHeartbeatDone(team, robot);
                    _running = false;
                    lock.unlock();
                    // awaken simworld
                    _conditionVar.notify_all();
                }
            };
        _thread = boost::thread(waitLoop);
    }

    bool isRunning() const { return _running; }
    void start() 
    { 
        assert(_running == false); // assert precondition, this allows lockless (re)start
        _running = true;
        // awaken lambda waiter thread
        _conditionVar.notify_all(); 
    }
private:
    bool _running = false;
    
    std::mutex _mutex;
    std::condition_variable _conditionVar;
    boost::thread _thread;
};

void waitForPut(RtDB2* rtdbConn, const std::string& rtdbKey)
{
    rtdbConn->waitForPut(rtdbKey);
}

Simworld::Simworld()
: _configAdapter(new RTDBConfigAdapter())
, _gameDataAdapter(new RTDBgameDataAdapter())
, _motionAdapter(new RTDBMotionAdapter())
, _timeAdapter(new RTDBTimeAdapter())
{
}

Simworld::~Simworld() {}

void Simworld::initialize(const std::string& arbiter, int sizeTeamA, int sizeTeamB)
{
    TRACE_FUNCTION("");

    _sizeTeamA = sizeTeamA;
    _sizeTeamB = sizeTeamB;
    if (arbiter == "autoref")
    {
        _autorefEnabled = true;
    }
    else
    {
        _autorefEnabled = false;
    }
    

    /* Seed the random number generator */
    std::srand(std::time(0));

    /* Sanity checks */
    if (_configAdapter == nullptr)
    {
        throw std::runtime_error("Error: configAdapter is null");
    }
    if (_gameDataAdapter == nullptr)
    {
        throw std::runtime_error("Error: gameDataAdapter is null");
    }
    if (_motionAdapter == nullptr)
    {
        throw std::runtime_error("Error: motionAdapter is null");
    }
    if (_timeAdapter == nullptr)
    {
        throw std::runtime_error("Error: timeAdapter is null");
    }

    /* Create initial simworld game data */
    try {
        auto gameData = GameDataFactory::createGameData(_sizeTeamA, _sizeTeamB);
        _simworldGameData = SimworldGameDataFactory::createCompleteWorld(gameData);
    }
    catch (std::exception& e) {
        std::cout << "Error creating initial game data: " << e.what() << std::endl;
    }

    /* Get tickFrequency and stepSize */
    try
    {
        _tickFrequency = _configAdapter->getTickFrequency();
        _tick_stepsize_s = (1.0 / _tickFrequency) * _configAdapter->getSimulationSpeedupFactor();
        _sleeptime_ms = (1000 / _tickFrequency) / (_sizeTeamA + _sizeTeamB + 2); // +1 for gameData, +1 for buffer to ensure we don't miss the simulation tick 
    }
    catch (std::exception& e) {
        std::cout << "Error obtaining tickFrequency and stepSize: " << e.what() << std::endl;
    }

    /* Publish non-robot game data */
    try {
        _gameDataAdapter->publishGameData(_simworldGameData);
    }
    catch (std::exception& e) {
        std::cout << "Error publishing non-robot game data: " << e.what() << std::endl;
    }

    /* Publish the game data, per robot */
    for (auto& teampair: _simworldGameData.team)
    {
        const auto teamID = teampair.first;
        for (auto& robotpair: teampair.second)
        {
            const auto robotID = robotpair.first;

            try {
                _gameDataAdapter->publishGameData(_simworldGameData, teamID, robotID);
            }
            catch (std::exception& e) {
                std::cout << "Error publishing initial robot game data: " << e.what() << std::endl;
            }
        }
    }

    if (_autorefEnabled)
    {
        _arbiter.initialize();
    }

    /* Publish initial scene */
    try
    {
        _gameDataAdapter->publishScene(_simworldGameData);
    }
    catch (std::exception& e)
    {
        std::cout << "Error publishing simulation scene: " << e.what() << std::endl;
    }

    /* Publish SIMULATION_TIME with time_now */
    try
    {
        // Get initial wallclock time, publish to SIMULATION_TIME to let ftime know time will be simulated
        _simulationTime = rtime::now();
        _timeAdapter->publishSimulationTime(_simulationTime);
    }
    catch (std::exception& e)
    {
        std::cout << "Error publishing initial simulation time: " << e.what() << std::endl;
    }

}

void Simworld::control()
{
    TRACE_FUNCTION("");

    while (true)
    {
        tprintf("Waiting for HEARTBEAT...");
        _timeAdapter->waitForHeartbeat();
        tprintf("Waking up from HEARTBEAT");


        /* Get tickFrequency and stepSize */
        try
        {
            _tickFrequency = _configAdapter->getTickFrequency();


            _tick_stepsize_s = (1.0 / _tickFrequency) * _configAdapter->getSimulationSpeedupFactor();


            // prevent divide by zero
            if (_tickFrequency == 0)
            {
                _sleeptime_ms = 0.0;
            }
            else
            {
                _sleeptime_ms = (1000 / _tickFrequency) / (_sizeTeamA + _sizeTeamB + 2 ); // +1 for thread join time limit, +1 for gameData, +1 for buffer to ensure we don't miss the simulation tick 
            }
        }
        catch (std::exception& e) {
            std::cout << "Error obtaining tickFrequency and stepSize: " << e.what() << std::endl;
        }


        /* A single 'tick' executes the following actions:
         * - Run the arbiter on the game data
         * - Get motion data, per robot
         * - Recalculate the game data
         *   -> Either by advancing time with _dt, or by loading a newly published simScene
         * - Advance and publish the simulated timestamp
         * - Publish non-robot game data
         * - WaitForPut for every robot to wait for heartbeat/tick to finish
         * - Publish the game data, per robot
         */

        /* Sanity checks */
        if (_configAdapter == nullptr)
        {
            throw std::runtime_error("Error: configAdapter is null");
        }
        if (_gameDataAdapter == nullptr)
        {
            throw std::runtime_error("Error: gameDataAdapter is null");
        }
        if (_motionAdapter == nullptr)
        {
            throw std::runtime_error("Error: motionAdapter is null");
        }

        /* Control the arbiter */
        if (_autorefEnabled)
        {
            auto arbiterGameData = _arbiter.control(_simworldGameData, _tick_stepsize_s);
            _simworldGameData.ball = arbiterGameData.ball;
        }

        /* Get motion data, per robot */
        for (auto& teampair: _simworldGameData.team)
        {
            const auto teamID = teampair.first;
            for (auto& robotpair: teampair.second)
            {
                const auto robotID = robotpair.first;

                try {
                    auto velocity = _motionAdapter->getVelocity(teamID, robotID);
                    _simworldGameData.team[teamID][robotID].setVelocityRCS(velocity);
                }
                catch (std::exception& e) {
                    /* Failed to get robot velocity, possibly data is too old. Reset velocity to 0 */
                    _simworldGameData.team[teamID][robotID].setVelocityRCS( Velocity2D(0.0, 0.0, 0.0) );

                    std::cout << "Error storing robot velocity: " << e.what() << std::endl;
                }

                try {
                    auto kicker = _motionAdapter->getKickerData(teamID, robotID);
                    _simworldGameData.team[teamID][robotID].setKickerHeight(kicker.height);
                    _simworldGameData.team[teamID][robotID].setKickerSpeed(kicker.speed, ROBOT_KICKER_SPEED_SCALING);
                }
                catch (std::exception& e) {
                    /* Failed to get kicker data, possibly data is too old. Reset kicker data to 0.0 */
                    _simworldGameData.team[teamID][robotID].setKickerSpeed(0.0, ROBOT_KICKER_SPEED_SCALING);

                    std::cout << "Error storing kicker data: " << e.what() << std::endl;
                }

                try {
                    if (_motionAdapter->hasBallHandlersEnabled(teamID, robotID))
                    {
                        _simworldGameData.team[teamID][robotID].enableBallHandlers();
                    }
                    else
                    {
                        _simworldGameData.team[teamID][robotID].disableBallHandlers();
                    }
                }
                catch (std::exception& e) {
                    std::cout << "Error storing ballhandler data: " << e.what() << std::endl;
                }
            }
        }

        /* Recalculate the game data */
        SimulationScene scene;
        if (_gameDataAdapter->checkUpdatedScene(scene))
        {
            // irregular 'reset', only if user manipulates the scene
            tprintf("overriding simulation scene");
            TRACE("overriding simulation scene");
            _simworldGameData.setScene(scene);
        }
        else
        {
            // normal update
            _simworldGameData.recalculateWorld(_tick_stepsize_s);
        }

        /* Advance and publish the simulated timestamp */
        try
        {
            _simulationTime += _tick_stepsize_s;
            _timeAdapter->publishSimulationTime(_simulationTime);
        }
        catch (std::exception& e)
        {
            std::cout << "Error publishing simulation time: " << e.what() << std::endl;
        }

        /* Publish non-robot game data */
        try {
            _gameDataAdapter->publishGameData(_simworldGameData);
        }
        catch (std::exception& e) {
            std::cout << "Error publishing non-robot game data: " << e.what() << std::endl;
        }

        {
            TRACE_SCOPE("SLEEP_GAMEDATA", "");
            std::this_thread::sleep_for(std::chrono::milliseconds(_sleeptime_ms));
        }

        /* WaitForPut for every robot to wait for heartbeat/tick to finish */
        /* Publish the game data, per robot */
        for (auto& teampair: _simworldGameData.team)
        {
            const auto teamID = teampair.first;
            for (const auto robotpair: teampair.second)
            {
                const auto robotID = robotpair.first;
                auto id = std::make_pair(teamID, robotID);
                auto itWaiter = _robotHeartbeatWaiters.find(id);
                if (itWaiter == _robotHeartbeatWaiters.end())
                {  // the waiter doesn't exist yet
                    _robotHeartbeatWaiters[id] = std::make_unique<HeartbeatWaiter>(teamID, robotID, *_timeAdapter);
                }
                else if (!itWaiter->second->isRunning())
                {
                    itWaiter->second->start();
                }
                else 
                { 
                    TRACE_ERROR("Robot %s (Team %s) heartbeat failed to finish within a tick", 
                        enum2str(robotID), enum2str(teamID));  
                }

                try {
                    _gameDataAdapter->publishGameData(_simworldGameData, teamID, robotID);
                }
                catch (std::exception& e) {
                    std::cout << "Error publishing robot game data team A: " << e.what() << std::endl;
                }
    
                {
                    /* Trace and sleep for this robot to spread robot computations over the available time */
                    TRACE_SCOPE("SLEEP_ROBOT", "");
                    std::this_thread::sleep_for(std::chrono::milliseconds(_sleeptime_ms));
                }
            }
        }

        // give robots time to finish the heartbeat / tick
        std::this_thread::sleep_for(std::chrono::milliseconds(_sleeptime_ms));

        /* Finally, publish SIMULATION_HEARTBEAT_DONE to notify execution that the heartbeat / tick for all robots has finished */
        _timeAdapter->publishSimulationHeartbeatDone();

        WRITE_TRACE;

    }
}
