// Copyright 2019-2022 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include "falconsCommon.hpp"
#include "tracing.hpp"

#include "RTDBOutputAdapter.hpp"

RTDBOutputAdapter::RTDBOutputAdapter()
{
    TRACE_FUNCTION("");

    int robot_id = getRobotNumber();
    rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(robot_id);
}

RTDBOutputAdapter::~RTDBOutputAdapter()
{
}

void RTDBOutputAdapter::setRobotVelocity(const ::Motion::RobotVector& velocity)
{
    TRACE_FUNCTION("");

    T_ROBOT_VELOCITY_FEEDBACK velocity_feedback;
    velocity_feedback.x = velocity.x();
    velocity_feedback.y = velocity.y();
    velocity_feedback.Rz = velocity.phi();
    rtdb->put(ROBOT_VELOCITY_FEEDBACK, &velocity_feedback);
}

void RTDBOutputAdapter::setRobotPosition(const ::Motion::RobotVector& position)
{
    TRACE_FUNCTION("");

    T_ROBOT_DISPLACEMENT_FEEDBACK displacement_feedback;
    displacement_feedback.x = position.x();
    displacement_feedback.y = position.y();
    displacement_feedback.Rz = position.phi();
    rtdb->put(ROBOT_DISPLACEMENT_FEEDBACK, &displacement_feedback);
}

void RTDBOutputAdapter::setBallHandlerAngles(const ::Motion::BallhandlerVector& angles)
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

void RTDBOutputAdapter::setMotorDiagnosticsOutput(const ::Motors::MotorVector &torque,
                                                  const ::Motors::MotorVector &velocity,
                                                  const ::Motors::MotorVector &velocity_demand,
                                                  const ::Motors::MotorVector &position,
                                                  const ::Motors::MotorVector &current,
                                                  const ::Motors::MotorVector &current_demand)
{
    TRACE_FUNCTION("");

    T_DIAG_PERIPHERALSINTERFACE diag;

    diag.torque_feedback[0] = torque.drive_motor_left();
    diag.torque_feedback[1] = torque.drive_motor_right();
    diag.torque_feedback[2] = torque.drive_motor_rear();

    diag.velocity_feedback[0] = velocity.drive_motor_left();
    diag.velocity_feedback[1] = velocity.drive_motor_right();
    diag.velocity_feedback[2] = velocity.drive_motor_rear();

    diag.velocity_setpoint[0] = velocity_demand.drive_motor_left();
    diag.velocity_setpoint[1] = velocity_demand.drive_motor_right();
    diag.velocity_setpoint[2] = velocity_demand.drive_motor_rear();

    // position.drive_motor_left();
    // position.drive_motor_right();
    // position.drive_motor_rear();

    diag.current_feedback[0] = current.drive_motor_left();
    diag.current_feedback[1] = current.drive_motor_right();
    diag.current_feedback[2] = current.drive_motor_rear();

    diag.current_setpoint[0] = current_demand.drive_motor_left();
    diag.current_setpoint[1] = current_demand.drive_motor_right();
    diag.current_setpoint[2] = current_demand.drive_motor_rear();

    rtdb->put(DIAG_PERIPHERALSINTERFACE, &diag);
}


void RTDBOutputAdapter::setMotorFeedback(const ::Motors::MotorVector &velocity)
{
    TRACE_FUNCTION("");

    T_MOTOR_FEEDBACK motor_feedback;

    motor_feedback.m1.velocity = velocity.drive_motor_left();
    motor_feedback.m2.velocity = velocity.drive_motor_right();
    motor_feedback.m3.velocity = velocity.drive_motor_rear();

    rtdb->put(MOTOR_FEEDBACK, &motor_feedback);
}