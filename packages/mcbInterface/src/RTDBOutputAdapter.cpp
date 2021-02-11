// Copyright 2019-2020 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "falconsCommon.hpp"
#include "tracing.hpp"

#include "RTDBOutputAdapter.hpp"

RTDBOutputAdapter::RTDBOutputAdapter()
{
    TRACE_FUNCTION("");

    int robot_id = getRobotNumber();
    rtdb = RtDB2Store::getInstance().getRtDB2(robot_id);
}

RTDBOutputAdapter::~RTDBOutputAdapter()
{
}

void RTDBOutputAdapter::setRobotVelocity(const ::motors::RobotVector& velocity)
{
    TRACE_FUNCTION("");

    T_ROBOT_VELOCITY_FEEDBACK velocity_feedback;
    velocity_feedback.x = velocity.x();
    velocity_feedback.y = velocity.y();
    velocity_feedback.Rz = velocity.phi();
    rtdb->put(ROBOT_VELOCITY_FEEDBACK, &velocity_feedback);
}

void RTDBOutputAdapter::setRobotPosition(const ::motors::RobotVector& position)
{
    TRACE_FUNCTION("");

    T_ROBOT_DISPLACEMENT_FEEDBACK displacement_feedback;
    displacement_feedback.x = position.x();
    displacement_feedback.y = position.y();
    displacement_feedback.Rz = position.phi();
    rtdb->put(ROBOT_DISPLACEMENT_FEEDBACK, &displacement_feedback);
}

void RTDBOutputAdapter::setBallHandlerAngles(const ::motors::BallhandlerAngles& angles)
{
    TRACE_FUNCTION("");

    T_BALLHANDLERS_FEEDBACK ballhandlers_feedback;
    ballhandlers_feedback.angleLeft = angles.left();
    ballhandlers_feedback.angleRight = angles.right();
    rtdb->put(BALLHANDLERS_FEEDBACK, &ballhandlers_feedback);
}

void RTDBOutputAdapter::setInPlayStatus(bool in_play)
{
    TRACE_FUNCTION("");

    T_INPLAY_FEEDBACK inplay_feedback;
    if (in_play) {
        inplay_feedback = robotStatusEnum::INPLAY;
    }
    else {
        inplay_feedback = robotStatusEnum::OUTOFPLAY;
    }

    rtdb->put(INPLAY_FEEDBACK, &inplay_feedback);
}
