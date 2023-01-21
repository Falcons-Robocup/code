// Copyright 2018-2021 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * simworld_main.cpp
 *
 *  Created on: Feb 19, 2019
 *      Author: Coen Tempelaars
 */

#include <chrono>
#include <stdexcept>
#include <thread>
#include <iostream>

#include "int/simulationCapabilities.hpp"
#include "int/simworld.hpp"
#include "tracing.hpp"


int main(int argc, char **argv)
{
    std::string arbiter = std::string( std::getenv("SIM_ARBITER") );
    int sizeTeamA = std::stoi( std::getenv("SIM_SIZE_TEAM_A") );
    int sizeTeamB = std::stoi( std::getenv("SIM_SIZE_TEAM_B") );

    try
    {
        INIT_TRACE("simulation");
        TRACE_FUNCTION("");

        Simworld simworld;
        simworld.initialize(arbiter, sizeTeamA, sizeTeamB);

        // control will:
        // - waitForPut HEARTBEAT
        // - advance simulation world by one tick
        // - advance the simulated timestamp
        // - publish new ROBOT_STATE
        simworld.control();
    }
    catch (std::exception &e)
    {
        std::cout << "Main loop caught exception: " << e.what() << std::endl;
    }
}
