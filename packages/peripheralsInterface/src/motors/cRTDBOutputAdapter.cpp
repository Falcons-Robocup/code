// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBOutputAdapter.cpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Erik Kouters
 */

#include "tracing.hpp"

#include "int/motors/cRTDBOutputAdapter.hpp"

cRTDBOutputAdapter::cRTDBOutputAdapter(PeripheralsInterfaceData& piData)
    : _piData(piData)
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId);
    TRACE("<");
}

cRTDBOutputAdapter::~cRTDBOutputAdapter()
{
}

void cRTDBOutputAdapter::setMotorFeedback()
{
    TRACE_FUNCTION("");

    T_MOTOR_FEEDBACK motorFeedback;

    piDisplacement disp = _piData.getDisplacementOutput();
    motorFeedback.m1.displacement = disp.m1_pos;
    motorFeedback.m2.displacement = disp.m2_pos;
    motorFeedback.m3.displacement = disp.m3_pos;

    piVelAcc vel = _piData.getVelocityOutput();
    motorFeedback.m1.velocity = vel.m1_vel;
    motorFeedback.m2.velocity = vel.m2_vel;
    motorFeedback.m3.velocity = vel.m3_vel;

    _rtdb->put(MOTOR_FEEDBACK, &motorFeedback);
}

void cRTDBOutputAdapter::setBallHandlersFeedback()
{
    TRACE_FUNCTION("");

    T_BALLHANDLERS_FEEDBACK ballHandlersFeedback;

    BallhandlerFeedback feedback = _piData.getBallhandlerFeedback();
    ballHandlersFeedback.angleLeft = feedback.angleLeft;
    ballHandlersFeedback.angleRight = feedback.angleRight;
    ballHandlersFeedback.velocityLeft = feedback.velocityLeft;
    ballHandlersFeedback.velocityRight = feedback.velocityRight;

    _rtdb->put(BALLHANDLERS_FEEDBACK, &ballHandlersFeedback);
}

void cRTDBOutputAdapter::setInPlayFeedback()
{
    TRACE_FUNCTION("");

    T_INPLAY_FEEDBACK inPlayFeedback;
    //_piData.

    _rtdb->put(INPLAY_FEEDBACK, &inPlayFeedback);
}
