 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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


static RTDBConfigAdapter rtdbConfigAdapter;
static RTDBgameDataAdapter rtdbGameDataAdapter;
static RTDBMotionAdapter rtdbMotionAdapter;
static RTDBTimeAdapter rtdbTimeAdapter;


Simworld::Simworld()
: _configAdapter(&rtdbConfigAdapter)
, _gameDataAdapter(&rtdbGameDataAdapter)
, _motionAdapter(&rtdbMotionAdapter)
, _timeAdapter(&rtdbTimeAdapter)
{
}

void Simworld::initialize()
{
    TRACE_FUNCTION("");

    _sizeTeamA = 0;
    _sizeTeamB = 0;

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
        _sizeTeamA = _configAdapter->getSize(TeamID::A);
        _sizeTeamB = _configAdapter->getSize(TeamID::B);
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
        _tick_stepsize_s = _configAdapter->getStepSizeMs() / 1000.0;
        _sleeptime_ms = (1000 / _tickFrequency) / (_sizeTeamA + _sizeTeamB + 1);
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

    if (_configAdapter->getArbiter() == "simworld")
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
        _timeAdapter->waitForSimulationTick();
        tprintf("Waking up from SIMULATION_TICK");


        /* Get tickFrequency and stepSize */
        try
        {
            _tickFrequency = _configAdapter->getTickFrequency();

            // prevent divide by zero
            if (_tickFrequency == 0)
            {
                continue;
            }

            _tick_stepsize_s = _configAdapter->getStepSizeMs() / 1000.0;
            _sleeptime_ms = (1000 / _tickFrequency) / (_sizeTeamA + _sizeTeamB + 1);
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
        if (_configAdapter->getArbiter() == "simworld")
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

        std::this_thread::sleep_for(std::chrono::milliseconds(_sleeptime_ms));

        /* Publish the game data, per robot */
        for (auto& teampair: _simworldGameData.team)
        {
            const auto teamID = teampair.first;
            for (const auto robotpair: teampair.second)
            {
                const auto robotID = robotpair.first;

                try {
                    _gameDataAdapter->publishGameData(_simworldGameData, teamID, robotID);
                }
                catch (std::exception& e) {
                    std::cout << "Error publishing robot game data team A: " << e.what() << std::endl;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(_sleeptime_ms));
            }
        }

    }
}

void Simworld::loop()
{

    while(true)
    {
        int tick_frequency = _configAdapter->getTickFrequency();

        //consider tick_frequency 0 as "paused"
        if (tick_frequency == 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        else
        {
            int period_ms = (1000 / tick_frequency);

            std::chrono::system_clock::time_point timePoint =
                std::chrono::system_clock::now() + std::chrono::milliseconds(period_ms);

            _timeAdapter->publishSimulationTick();
            WRITE_TRACE;

            std::this_thread::sleep_until(timePoint);
        }
    }
}
