// Copyright 2019-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBOutputAdapter.cpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#include "int/adapters/RTDBOutputAdapter.hpp"

#include "falconsCommon.hpp" //getRobotNumber(), getTeamChar()
#include "tracing.hpp"

RTDBOutputAdapter::RTDBOutputAdapter(bool verbose, int robotId)
{
    TRACE_FUNCTION("");
    _verbose = verbose;
    _myRobotId = robotId;
    auto teamChar = getTeamChar();
    _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(_myRobotId, teamChar);
}

RTDBOutputAdapter::~RTDBOutputAdapter()
{
}

void RTDBOutputAdapter::setSubtarget(actionResultTypeEnum const &status, robotPosVel const &robotPosVelSetpoint)
{
    TRACE_FUNCTION("");

    if (_verbose)
    {
        tprintf("put result=%-8s  ROBOT_POSVEL_SETPOINT=( pos=[%6.2f, %6.2f, %6.2f], vel=[%6.2f, %6.2f, %6.2f], motionType=%s )", enum2str(status), robotPosVelSetpoint.position.x, robotPosVelSetpoint.position.y, robotPosVelSetpoint.position.Rz, robotPosVelSetpoint.velocity.x, robotPosVelSetpoint.velocity.y, robotPosVelSetpoint.velocity.Rz, enum2str(robotPosVelSetpoint.motionType));
    }

    _rtdb->put(ROBOT_POSVEL_SETPOINT, &robotPosVelSetpoint);
}

void RTDBOutputAdapter::setDiagnostics(diagPathPlanning const &diagnostics)
{
    TRACE_FUNCTION("");
    // for consistency when inspecting tracing: trace the entire struct as well
    TRACE("diagnostics: %s", tostr(diagnostics).c_str());
    // write to RTDB
    _rtdb->put(DIAG_PATHPLANNING, &diagnostics);
}

