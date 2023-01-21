// Copyright 2019-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBOutputAdapter.cpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Erik Kouters
 */

#include "tracing.hpp"
#include "falconsCommon.hpp" //getRobotNumber()

#include "int/adapters/cRTDBOutputAdapter.hpp"

cRTDBOutputAdapter::cRTDBOutputAdapter()
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(_myRobotId);
    TRACE("<");
}

cRTDBOutputAdapter::~cRTDBOutputAdapter()
{
}

void cRTDBOutputAdapter::setBallHandlersMotorSetpoint(const bool& enabled, const ballHandlersSetpointsType& setpoints)
{
    TRACE_FUNCTION("");

    T_BALLHANDLERS_MOTOR_SETPOINT ballHandlersMotorSetpoint;
    ballHandlersMotorSetpoint.enabled = enabled;
    ballHandlersMotorSetpoint.bhMotorData.angleLeft = setpoints.angleLeft;
    ballHandlersMotorSetpoint.bhMotorData.angleRight = setpoints.angleRight;
    ballHandlersMotorSetpoint.bhMotorData.velocityLeft = setpoints.velocityLeft;
    ballHandlersMotorSetpoint.bhMotorData.velocityRight = setpoints.velocityRight;

    _rtdb->put(BALLHANDLERS_MOTOR_SETPOINT, &ballHandlersMotorSetpoint);
}

void cRTDBOutputAdapter::setBallHandlersBallPossession(const bool& ballPossession)
{
    TRACE_FUNCTION("");

    _rtdb->put(BALLHANDLERS_BALL_POSSESSION, &ballPossession);
}

void cRTDBOutputAdapter::setDiagnostics(DiagBallHandling const &diag)
{
    TRACE_FUNCTION("");
    _rtdb->put(DIAG_BALLHANDLING, &diag);
}

