// Copyright 2019-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * PathPlanningClient.cpp
 *
 *  Created on: December, 2019
 *      Author: Jan Feitsma
 */


#include "ext/PathPlanningClient.hpp"
#include "int/PathPlanning.hpp"
#include "int/adapters/RTDBInputAdapter.hpp"
#include "int/adapters/RTDBOutputAdapter.hpp"

// common Falcons headers
#include "ConfigRTDBAdapter.hpp" // configuration
#include "tracing.hpp" // tracing
#include "falconsCommon.hpp" // common


// globals... better to put in this file, instead of making everything
// visible to motionPlanning via PathPlanningClient.hpp
RTDBInputAdapter *g_rtdbInputAdapter;
RTDBOutputAdapter *g_rtdbOutputAdapter;
ConfigRTDBAdapter<ConfigPathPlanning> *g_configAdapterPP;
ConfigRTDBAdapter<ConfigExecution> *g_configAdapterEx; // tick frequency
PathPlanning *g_pp;


PathPlanningClient::PathPlanningClient()
{
    // setup adapters
    TRACE("setting up RTDBInputAdapter");
    g_rtdbInputAdapter = new RTDBInputAdapter();

    TRACE("setting up RTDBOutputAdapter");
    g_rtdbOutputAdapter = new RTDBOutputAdapter(true);

    TRACE("setting up ConfigRTDBAdapterPP");
    g_configAdapterPP = new ConfigRTDBAdapter<ConfigPathPlanning>(CONFIG_PATHPLANNING);
    std::string configFile = determineConfig("PathPlanning");
    g_configAdapterPP->loadYAML(configFile);

    TRACE("setting up ConfigRTDBAdapterEx");
    g_configAdapterEx = new ConfigRTDBAdapter<ConfigExecution>(CONFIG_EXECUTION);
    configFile = determineConfig("execution");
    g_configAdapterEx->loadYAML(configFile);

    // setup PathPlanning, connect adapters
    TRACE("setting up PathPlanning");
    g_pp = new PathPlanning(g_configAdapterPP, g_configAdapterEx, g_rtdbInputAdapter, g_rtdbOutputAdapter);
}

PathPlanningClient::~PathPlanningClient()
{
    delete g_pp;
    delete g_configAdapterPP;
    delete g_configAdapterEx;
    delete g_rtdbOutputAdapter;
    delete g_rtdbInputAdapter;
}

actionResultTypeEnum PathPlanningClient::iterate()
{
    if (g_pp != NULL)
    {
        actionResultTypeEnum result = g_pp->iterate();
        return result;
    }
    return actionResultTypeEnum::FAILED;
}

void PathPlanningClient::spin()
{
    if (g_rtdbInputAdapter == NULL) return;
    if (g_pp == NULL) return;
    // run
    TRACE("starting spin");
    bool shutdown = false;
    while (!shutdown)
    {
        TRACE("wait");
        g_rtdbInputAdapter->waitForMotionSetpoint();
        TRACE("iterate");
        (void)g_pp->iterate(); // ignore output ...
        // NOTE: this function was used when pathPlanning ran as its own process
        // but now motionPlanning calls pathPlanning as library via iterate()
        // so it can use its return feedback PASSED/RUNNING/FAILED
    }
    TRACE("done");
}

