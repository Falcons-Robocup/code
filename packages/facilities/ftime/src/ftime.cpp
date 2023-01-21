// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ftime.cpp
 *
 * Falcons timestamp utilities, allowing for simulated time
 *
 *  Created on: June, 2020
 *      Author: Erik Kouters
 */

#include "ftime.hpp"

#include "falconsCommonLegacy.hpp"

rtime ftime::now()
{
    // Check if time is simulated

    auto rtdbConnection = FalconsRTDBStore::getInstance().getFalconsRTDB(COACH_AGENTID);
    
    T_SIMULATION_TIME simTime;

    auto r = rtdbConnection->get(SIMULATION_TIME, &simTime, COACH_AGENTID);
    if (r == RTDB2_SUCCESS)
    {
        // Time is indeed simulated
        //tprintf("get SIMULATION_TIME simTime=%f", simTime);

        return rtime(simTime);
    }
    else
    {
        // Time is not simulated

        return rtime::now();
    }
}

