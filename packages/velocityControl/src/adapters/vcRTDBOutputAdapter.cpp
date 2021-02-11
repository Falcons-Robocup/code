// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * vcRTDBOutputAdapter.cpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#include "int/adapters/vcRTDBOutputAdapter.hpp"

#include "falconsCommon.hpp" //getRobotNumber(), getTeamChar()
#include "tracing.hpp"

vcRTDBOutputAdapter::vcRTDBOutputAdapter(bool verbose, int robotId)
{
    TRACE_FUNCTION("");
    _verbose = verbose;
    _myRobotId = robotId;
    auto teamChar = getTeamChar();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId, teamChar);
}

vcRTDBOutputAdapter::~vcRTDBOutputAdapter()
{
}

void vcRTDBOutputAdapter::setVelocity(robotVelocity const &robotVelocitySetpoint)
{
    TRACE_FUNCTION("");

    if (_verbose)
    {
        tprintf("put ROBOT_VELOCITY_SETPOINT=[%6.2f, %6.2f, %6.2f]", robotVelocitySetpoint.x, robotVelocitySetpoint.y, robotVelocitySetpoint.Rz);
    }

    _rtdb->put(ROBOT_VELOCITY_SETPOINT, &robotVelocitySetpoint);
}

void vcRTDBOutputAdapter::setDiagnostics(diagVelocityControl const &diagnostics)
{
    TRACE_FUNCTION("");
    // for consistency when inspecting tracing: trace the entire struct as well
    TRACE("diagnostics: %s", tostr(diagnostics).c_str());
    // write to RTDB
    _rtdb->put(DIAG_VELOCITYCONTROL, &diagnostics);
}

