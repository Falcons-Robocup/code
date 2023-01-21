// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBTimeAdapter.cpp
 *
 *  Created on: June 07, 2020
 *      Author: Erik Kouters
 */

#include "int/RTDBtimeAdapter.hpp"
#include "int/RTDBaccess.hpp"

#include <stdexcept>

#include "tracing.hpp"

void RTDBTimeAdapter::waitForHeartbeat() const
{
    TRACE_FUNCTION("");
    
    auto rtdbConnection = getRTDBConnection();
    auto r = rtdbConnection->waitForPut(HEARTBEAT);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error during WaitForPut HEARTBEAT");
    }
}

void RTDBTimeAdapter::publishSimulationTime (const rtime& simTime) const
{
    TRACE_FUNCTION("");
    T_SIMULATION_TIME rtdbSimTime = simTime.toDouble();

    auto rtdbConnection = getRTDBConnection();

    tprintf("put SIMULATION_TIME = %f", rtdbSimTime);
    auto r = rtdbConnection->put(SIMULATION_TIME, &rtdbSimTime);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error writing SIMULATION_TIME to RtDB");
    }
}

void RTDBTimeAdapter::publishSimulationHeartbeatDone() const
{
    TRACE_FUNCTION("");
    T_SIMULATION_HEARTBEAT_DONE simHBDone = 0;

    auto rtdbConnection = getRTDBConnection();
    auto r = rtdbConnection->put(SIMULATION_HEARTBEAT_DONE, &simHBDone);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error writing SIMULATION_HEARTBEAT_DONE to RtDB");
    }
}

void RTDBTimeAdapter::waitForPutHeartbeatDone(TeamID teamID, RobotID robotID) const
{
    auto rtdbConnection = getRTDBConnection(teamID, robotID);
    rtdbConnection->waitForPut(ACTION_RESULT);
}