// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * main.cpp
 *
 *  Created on: Mar 4, 2018
 *      Author: Jan Feitsma
 */


#include <boost/thread.hpp>

#include "int/ballHandlingControl.hpp"

#include "int/adapters/cRTDBInputAdapter.hpp"
#include "int/adapters/cRTDBOutputAdapter.hpp"
#include "ConfigRTDBAdapter.hpp"

#include "cDiagnostics.hpp"
#include "tracing.hpp"

boost::thread _workerThreadWaitForBallHandlersFeedback;


int main(int argc, char **argv)
{
    INIT_TRACE;

    try
    {
        // setup adapters
        TRACE("setup adapters");
        cRTDBInputAdapter rtdbInputAdapter;
        cRTDBOutputAdapter rtdbOutputAdapter;
        ConfigRTDBAdapter<ConfigBallHandling> configAdapter(CONFIG_BALLHANDLING);
        std::string configFile = determineConfig("BallHandling");
        configAdapter.loadYAML(configFile);
        TRACE("done setup adapters");
        WRITE_TRACE;

        // control algorithm
        TRACE("setup ballHandlingControl");
        ballHandlingControl controller(rtdbOutputAdapter, &configAdapter);
        TRACE("done setup ballHandlingControl");
        WRITE_TRACE;

        rtdbInputAdapter = cRTDBInputAdapter(&controller);

        // wait for ballhandlers feedback in a separate thread
        _workerThreadWaitForBallHandlersFeedback = boost::thread(boost::bind(&cRTDBInputAdapter::waitForBallHandlersFeedback, &rtdbInputAdapter));

        // spin
        TRACE("spin");
        rtdbInputAdapter.waitForBallHandlersSetpoint();
        
        // cleanup
    }
    catch (std::exception &e)
    {
        std::cerr << "Error occurred:" << e.what() << std::endl;
        TRACE_ERROR("Error occurred: %s", e.what());
        return 1;
    }
    return 0;
}

