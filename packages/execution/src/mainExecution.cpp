// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 *
 * Maintains the system's heartbeat.
 * A new heartbeat is triggered with configured frequency, iff the previous heartbeat has finished.
 * To determine the previous heartbeat has finished, this process will subscribe (waitForPut) to a configured RtDB key.
 *
 */

#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "FalconsRTDB.hpp"
#include "ConfigRTDBAdapter.hpp"
#include "falconsCommon.hpp"
#include "tracing.hpp"

int main(int argc, char **argv)
{
    try
    {

        INIT_TRACE("execution");

        int myRobotId = getRobotNumber();
        auto teamChar = getTeamChar();
        auto rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(myRobotId, teamChar);

        // Fetch configuration from yaml file
        std::string configFile = determineConfig("execution");
        ConfigRTDBAdapter<T_CONFIG_EXECUTION>* configAdapter = new ConfigRTDBAdapter<T_CONFIG_EXECUTION>(CONFIG_EXECUTION);
        configAdapter->loadYAML(configFile);

        T_CONFIG_EXECUTION config;

        /*
         * Create loop for heart beat generation
         */
        while(true)
        {
            TRACE_SCOPE("HEARTBEAT", "");
            // Get configuration data inside loop to allow live update
            configAdapter->get(config);
            float sleepMs = 1000.0 / config.frequency;

            std::chrono::system_clock::time_point end_time = std::chrono::system_clock::now() + std::chrono::milliseconds((int)sleepMs);

            T_HEARTBEAT beat = 0;
            rtdb->put(HEARTBEAT, &beat);

            // Sleep to force frequency
            std::this_thread::sleep_until(end_time);

            WRITE_TRACE;
        }

        delete configAdapter;
    }
    catch (std::exception &e)
    {
    	throw e;
    }

}
