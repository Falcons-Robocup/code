// Copyright 2019-2020 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include <stdexcept>

#include "falconsCommon.hpp"
#include "tracing.hpp"

#include "RTDBInputAdapter.hpp"

using std::runtime_error;

RTDBInputAdapter::RTDBInputAdapter()
{
    TRACE_FUNCTION("");

    int robot_id = getRobotNumber();
    rtdb = RtDB2Store::getInstance().getRtDB2(robot_id);
}

RTDBInputAdapter::~RTDBInputAdapter()
{
}

::motors::RobotVector RTDBInputAdapter::getRobotVelocitySetpoint()
{
    TRACE_FUNCTION("");

    T_ROBOT_VELOCITY_SETPOINT robot_velocity_setpoint;
    int r = rtdb->get(ROBOT_VELOCITY_SETPOINT, &robot_velocity_setpoint);
    if (r != RTDB2_SUCCESS) {
        throw(runtime_error("Failed to retrieve robot velocity setpoint."));
    }

    ::motors::RobotVector velocity_setpoint;
    velocity_setpoint.set_x(robot_velocity_setpoint.x);
    velocity_setpoint.set_y(robot_velocity_setpoint.y);
    velocity_setpoint.set_phi(robot_velocity_setpoint.Rz);

    return velocity_setpoint;
}

::motors::BallhandlerAngles RTDBInputAdapter::getBallhandlerAngleSetpoints()
{
    TRACE_FUNCTION("");

    T_BALLHANDLERS_MOTOR_SETPOINT ballhandler_motor_setpoint;
    int r = rtdb->get(BALLHANDLERS_MOTOR_SETPOINT, &ballhandler_motor_setpoint);
    if (r != RTDB2_SUCCESS) {
        throw(runtime_error("Failed to retrieve ballhandler motor setpoint."));
    }

    ::motors::BallhandlerAngles ballhandler_angles;
    ballhandler_angles.set_left(ballhandler_motor_setpoint.bhMotorData.angleLeft);
    ballhandler_angles.set_right(ballhandler_motor_setpoint.bhMotorData.angleRight);

    return ballhandler_angles;
}

bool RTDBInputAdapter::getBallhandlersOn()
{
    TRACE_FUNCTION("");

    T_BALLHANDLERS_MOTOR_SETPOINT ballhandler_motor_setpoint;
    int r = rtdb->get(BALLHANDLERS_MOTOR_SETPOINT, &ballhandler_motor_setpoint);
    if (r != RTDB2_SUCCESS) {
        throw(runtime_error("Failed to retrieve ballhandler motor setpoint."));
    }

    return ballhandler_motor_setpoint.enabled;
}

void RTDBInputAdapter::waitForKickerSetpoint()
{
    rtdb->waitForPut(KICKER_SETPOINT);
}

KickerSetpoint RTDBInputAdapter::getKickerSetpoint()
{
    TRACE_FUNCTION("");

    T_KICKER_SETPOINT kicker_setpoint;
    int r = rtdb->get(KICKER_SETPOINT, &kicker_setpoint);
    if (r != RTDB2_SUCCESS) {
        throw(runtime_error("Failed to retrieve kicker setpoint."));
    }

    KickerSetpoint setpoint;
    switch (kicker_setpoint.kickerSetpointType)
    {
        case kickerSetpointTypeEnum::HOME:
        setpoint.type = KickerSetpoint::Type::Home;
            break;
        case kickerSetpointTypeEnum::SET_HEIGHT:
        setpoint.type = KickerSetpoint::Type::SetHeight;
        setpoint.value = kicker_setpoint.kickerHeight / 255.0;
            break;
        case kickerSetpointTypeEnum::SHOOT:
        setpoint.type = KickerSetpoint::Type::Shoot;
        setpoint.value = kicker_setpoint.kickerPower / 255.0;
            break;
    }

    return setpoint;
}
