// Copyright 2019-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBOutputAdapter.cpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Erik Kouters
 */

#include "tracing.hpp"
#include "cDiagnostics.hpp"
#include "falconsCommon.hpp" //getRobotNumber(), getTeamChar()

#include "int/adapters/cRTDBOutputAdapter.hpp"

cRTDBOutputAdapter::cRTDBOutputAdapter()
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    auto teamChar = getTeamChar();
    tprintf("cRTDBOutputAdapter get RTDB start");
    _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(_myRobotId, teamChar);
    tprintf("cRTDBOutputAdapter get RTDB end");
    TRACE("<");
}

cRTDBOutputAdapter::~cRTDBOutputAdapter()
{
}

void cRTDBOutputAdapter::setKickerSetpoint(kickerSetpointTypeEnum kickerSetpointType, float kickerHeight, float kickerPower)
{
    TRACE_FUNCTION("");

    T_KICKER_SETPOINT kickerSetpoint;
    kickerSetpoint.kickerSetpointType = kickerSetpointType;
    kickerSetpoint.kickerHeight = kickerHeight;
    kickerSetpoint.kickerPower = kickerPower;

    tprintf("kicking with setpoint power=%6.2f height=%6.2f", kickerPower, kickerHeight);
    if (kickerPower > 0)
    {
        TRACE_INFO("kicking with speed %.1f and height %.1f", kickerPower, kickerHeight);
    }

    _rtdb->put(KICKER_SETPOINT, &kickerSetpoint);
}
