// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * main.cpp
 *
 *  Created on: Nov 10, 2015
 *      Author: Tim Kouters
 */

#include <stdexcept>
#include <chrono>
#include <exception>
#include <functional>
#include <thread>

#include "ConfigRTDBAdapter.hpp"
#include "cDiagnostics.hpp"
#include "falconsCommon.hpp"
#include "FalconsRTDB.hpp"

using std::exception;

int main(int argc, char **argv)
{
    try
    {
        int myRobotId = getRobotNumber();
        auto teamChar = getTeamChar();
        auto rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(myRobotId, teamChar);

        // Fetch configuration from yaml file
        std::string configFile = determineConfig("heartBeatCoach");
        ConfigRTDBAdapter<T_CONFIG_HEARTBEATCOACH>* configAdapter = new ConfigRTDBAdapter<T_CONFIG_HEARTBEATCOACH>(CONFIG_HEARTBEATCOACH);
        configAdapter->loadYAML(configFile);

        T_CONFIG_HEARTBEATCOACH config;

        /*
         * Create loop for heart beat generation
         */
        while(true)
        {
            // Get configuration data inside loop to allow live update
            configAdapter->get(config);
            float sleepMs = 1000.0 / config.updateFrequency;

            std::chrono::system_clock::time_point end_time = std::chrono::system_clock::now() + std::chrono::milliseconds((int)sleepMs);

            //TRACE("heartbeat LATENCY start");
            T_HEARTBEAT_COACH beat = 0;
            rtdb->put(HEARTBEAT_COACH, &beat);

            std::this_thread::sleep_until(end_time);
        }

        delete configAdapter;
    }
    catch (exception &e)
    {
    	throw e;
    }

}
