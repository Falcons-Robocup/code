// Copyright 2020 Erik Kouters (Falcons)
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

void RTDBTimeAdapter::waitForSimulationTick() const
{
    TRACE_FUNCTION("");
    
    auto rtdbConnection = getRTDBConnection();
    rtdbConnection->waitForPut(SIMULATION_TICK);
}

void RTDBTimeAdapter::publishSimulationTick() const
{
    TRACE_FUNCTION("");
    
    auto rtdbConnection = getRTDBConnection();
    T_SIMULATION_TICK tick = 0;
    rtdbConnection->put(SIMULATION_TICK, &tick);
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
