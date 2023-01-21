// Copyright 2019-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBInputAdapter.cpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Erik Kouters
 */

#include "tracing.hpp"

#include "int/adapters/cRTDBInputAdapter.hpp"

cRTDBInputAdapter::cRTDBInputAdapter(ballHandlingControl *bhControl)
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(_myRobotId);
    _bhControl = bhControl;
    TRACE("<");
}

cRTDBInputAdapter::~cRTDBInputAdapter()
{
}

void cRTDBInputAdapter::waitForBallHandlersSetpoint()
{
    while (true)
    {
        _rtdb->waitForPut(BALLHANDLERS_SETPOINT);
        getBallHandlersSetpoint();

        _bhControl->updateSetpoint();
    }
}

void cRTDBInputAdapter::waitForBallHandlersFeedback()
{
    INIT_TRACE_THREAD("waitForBallHandlersFeedback");

    while (true)
    {
        _rtdb->waitForPut(BALLHANDLERS_FEEDBACK);
        getBallHandlersFeedback();
        getRobotVelocitySetpoint();

        _bhControl->updateSetpoint();
        _bhControl->updateFeedback();
    }
}

void cRTDBInputAdapter::getBallHandlersSetpoint()
{
    TRACE_FUNCTION("");
    T_BALLHANDLERS_SETPOINT ballHandlersSetpoint;

    int r = _rtdb->get(BALLHANDLERS_SETPOINT, &ballHandlersSetpoint);

    if (r == RTDB2_SUCCESS)
    {
        _bhControl->update_enabled(ballHandlersSetpoint);
    }
}

void cRTDBInputAdapter::getBallHandlersFeedback()
{
    TRACE_FUNCTION("");
    T_BALLHANDLERS_FEEDBACK ballHandlersFeedback;

    int r = _rtdb->get(BALLHANDLERS_FEEDBACK, &ballHandlersFeedback);

    if (r == RTDB2_SUCCESS)
    {
        ballHandlersStatusType bhStatus;
        bhStatus.angleLeft = ballHandlersFeedback.angleLeft;
        bhStatus.angleRight = ballHandlersFeedback.angleRight;
        bhStatus.velocityLeft = ballHandlersFeedback.velocityLeft;
        bhStatus.velocityRight = ballHandlersFeedback.velocityRight;

        _bhControl->update_status(bhStatus);
    }
}

void cRTDBInputAdapter::getRobotVelocityFeedback()
{
    TRACE_FUNCTION("");
    T_ROBOT_VELOCITY_FEEDBACK robotVelocityFeedback;

    int r = _rtdb->get(ROBOT_VELOCITY_FEEDBACK, &robotVelocityFeedback);

    if (r == RTDB2_SUCCESS)
    {
        Velocity2D robotVel = Velocity2D(robotVelocityFeedback.x, robotVelocityFeedback.y, robotVelocityFeedback.Rz);

        _bhControl->update_robot_velocity(robotVel);
    }

}

void cRTDBInputAdapter::getRobotVelocitySetpoint()
{
    TRACE_FUNCTION("");
    T_ROBOT_VELOCITY_SETPOINT robotVelocitySetpoint;

    int r = _rtdb->get(ROBOT_VELOCITY_SETPOINT, &robotVelocitySetpoint);

    if (r == RTDB2_SUCCESS)
    {
        Velocity2D robotVel = Velocity2D(robotVelocitySetpoint.x, robotVelocitySetpoint.y, robotVelocitySetpoint.Rz);

        _bhControl->update_robot_velocity(robotVel);
    }
}