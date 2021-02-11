// Copyright 2018-2020 Coen Tempelaars (Falcons)
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
    try
    {
        INIT_TRACE;
        TRACE_FUNCTION("");

        Simworld simworld;
        simworld.initialize();

        // control will:
        // - waitForPut SIMULATION_TICK
        // - advance simulation world by one tick
        // - advance the simulated timestamp
        // - publish new ROBOT_STATE
        std::thread simworldControlThread = std::thread(&Simworld::control, &simworld);

        // loop will:
        // - get simulation tick frequency (e.g., 20hz)
        // - sleep to maintain 20hz frequency
        // - put SIMULATION_TICK
        simworld.loop();
    }
    catch(std::exception& e)
    {
        std::cout << "Main loop caught exception: " << e.what() << std::endl;
    }
}
