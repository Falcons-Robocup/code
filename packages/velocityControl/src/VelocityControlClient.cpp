// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * VelocityControlClient.cpp
 *
 *  Created on: December, 2019
 *      Author: Jan Feitsma
 */


#include "ext/VelocityControlClient.hpp"
#include "int/VelocityControl.hpp"
#include "int/adapters/vcRTDBInputAdapter.hpp"
#include "int/adapters/vcRTDBOutputAdapter.hpp"
#include "int/adapters/vcConfigRTDBAdapter.hpp" // configuration

// common Falcons headers
#include "tracing.hpp" // tracing
#include "falconsCommon.hpp" // common


// globals... better to put in this file, instead of making everything
// visible to pathPlanning via VelocityControlClient.hpp
vcRTDBInputAdapter *g_vcRtdbInputAdapter;
vcRTDBOutputAdapter *g_vcRtdbOutputAdapter;
vcConfigRTDBAdapter *g_vcConfigAdapter;
ConfigRTDBAdapter<ConfigPathPlanning> *g_ppConfigAdapter;
VelocityControl *g_vc;


VelocityControlClient::VelocityControlClient()
{
    INIT_TRACE;

    // setup adapters
    TRACE("setting up vcRTDBInputAdapter");
    g_vcRtdbInputAdapter = new vcRTDBInputAdapter();

    TRACE("setting up vcRTDBOutputAdapter");
    g_vcRtdbOutputAdapter = new vcRTDBOutputAdapter(true);

    TRACE("setting up ConfigRTDBAdapter<VelocityControl>");
    g_vcConfigAdapter = new vcConfigRTDBAdapter(CONFIG_VELOCITYCONTROL);
    std::string vcConfigFile = determineConfig("VelocityControl");
    g_vcConfigAdapter->loadYAML(vcConfigFile);

    TRACE("setting up ConfigRTDBAdapter<PathPlanning>");
    g_ppConfigAdapter = new ConfigRTDBAdapter<ConfigPathPlanning>(CONFIG_PATHPLANNING);
    std::string ppConfigFile = determineConfig("PathPlanning");
    g_ppConfigAdapter->loadYAML(ppConfigFile);

    // setup VelocityControl, connect adapters
    TRACE("setting up VelocityControl");
    g_vc = new VelocityControl(g_vcConfigAdapter, g_ppConfigAdapter, g_vcRtdbInputAdapter, g_vcRtdbOutputAdapter);
}

VelocityControlClient::~VelocityControlClient()
{
    delete g_vc;
    delete g_vcConfigAdapter;
    delete g_vcRtdbOutputAdapter;
    delete g_vcRtdbInputAdapter;
}

void VelocityControlClient::iterate()
{
    if (g_vc != NULL)
    {
        g_vc->iterate();
    }
}

void VelocityControlClient::spin()
{
    if (g_vcRtdbInputAdapter == NULL) return;
    if (g_vc == NULL) return;
    // run
    TRACE("starting spin");
    bool shutdown = false;
    while (!shutdown)
    {
        TRACE("wait");
        g_vcRtdbInputAdapter->waitForRobotPosVelSetpoint();
        TRACE("iterate");
        (void)g_vc->iterate(); // ignore output ...
    }
    TRACE("done");
}

