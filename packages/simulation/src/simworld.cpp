 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
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
#include "int/simulationCapabilities.hpp"
#include "int/simworldGameDataFactory.hpp"
#include "tracing.hpp"

// The simulator must sleep after serving a robot (max. 10) or the arbiter (max. 1)
const static int SLEEPTIME_MS = (1000 / SIMULATION_FREQUENCY) / 11;

static RTDBConfigAdapter rtdbConfigAdapter;
static RTDBgameDataAdapter rtdbGameDataAdapter;
static RTDBMotionAdapter rtdbMotionAdapter;


Simworld::Simworld()
: _configAdapter(&rtdbConfigAdapter)
, _gameDataAdapter(&rtdbGameDataAdapter)
, _motionAdapter(&rtdbMotionAdapter)
{
}

void Simworld::initialize()
{
    TRACE_FUNCTION("");

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

    /* Create initial simworld game data */
    try {
        auto sizeTeamA = _configAdapter->getSize(TeamID::A);
        auto sizeTeamB = _configAdapter->getSize(TeamID::B);
        auto gameData = GameDataFactory::createGameData(sizeTeamA, sizeTeamB);
        _simworldGameData = SimworldGameDataFactory::createCompleteWorld(gameData);
    }
    catch (std::exception& e) {
        std::cout << "Error creating initial game data: " << e.what() << std::endl;
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
}

void Simworld::control()
{
    TRACE_FUNCTION("");

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
        auto arbiterGameData = _arbiter.control(_simworldGameData);
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
                std::cout << "Error storing robot velocity: " << e.what() << std::endl;
            }

            try {
                auto kicker = _motionAdapter->getKickerData(teamID, robotID);
                _simworldGameData.team[teamID][robotID].setKickerHeight(kicker.height);
                _simworldGameData.team[teamID][robotID].setKickerSpeed(kicker.speed, ROBOT_KICKER_SPEED_SCALING);
            }
            catch (std::exception& e) {
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
    _simworldGameData.recalculateWorld(SIMULATION_PERIOD);

    /* Publish non-robot game data */
    try {
        _gameDataAdapter->publishGameData(_simworldGameData);
    }
    catch (std::exception& e) {
        std::cout << "Error publishing non-robot game data: " << e.what() << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(SLEEPTIME_MS));

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

            std::this_thread::sleep_for(std::chrono::milliseconds(SLEEPTIME_MS));
        }
    }
}
