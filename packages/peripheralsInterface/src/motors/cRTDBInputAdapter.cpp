// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBInputAdapter.cpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Erik Kouters
 */

#include "tracing.hpp"

#include "int/motors/cRTDBInputAdapter.hpp"

cRTDBInputAdapter::cRTDBInputAdapter(PeripheralsInterfaceData& piData) :
    _piData(piData)
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(_myRobotId);
    TRACE("<");
}

cRTDBInputAdapter::~cRTDBInputAdapter()
{
}

void cRTDBInputAdapter::getMotorVelocitySetpoint()
{
    TRACE_FUNCTION("");
    T_MOTOR_VELOCITY_SETPOINT motorVelSetpoint;
    int ageMs = 0;

    int r = _rtdb->get(MOTOR_VELOCITY_SETPOINT, &motorVelSetpoint, ageMs, _myRobotId);
    int watchDogTimeoutMs = 150;

    // we need watchdog behavior in case any process breaks the execution architecture stream
    // motor velocity setpoint would freeze in that case, so we should to not relay it to motors, better to hit the brakes then!
    piVelAcc vel;
    vel.m1_vel = 0.0;
    vel.m2_vel = 0.0;
    vel.m3_vel = 0.0;
    if ((r == RTDB2_SUCCESS) && (ageMs < watchDogTimeoutMs))
    {
        vel.m1_vel = motorVelSetpoint.m1;
        vel.m2_vel = motorVelSetpoint.m2;
        vel.m3_vel = motorVelSetpoint.m3;

    }
    _piData.setVelocityInput(vel);
}

void cRTDBInputAdapter::getBallHandlersMotorSetpoint()
{
    TRACE_FUNCTION("");
    T_BALLHANDLERS_MOTOR_SETPOINT ballHandlersMotorSetpoint;
    int ageMs = 0;

    int r = _rtdb->get(BALLHANDLERS_MOTOR_SETPOINT, &ballHandlersMotorSetpoint, ageMs, _myRobotId);

    if (r == RTDB2_SUCCESS)
    {
        // BH Enabled / Disabled
        BallhandlerSettings settings = _piData.getBallhandlerSettings();
        if (ballHandlersMotorSetpoint.enabled)
        {
            settings.controlMode = BallhandlerBoardControlMode::BALLHANDLER_CONTROL_MODE_ON;
        }
        else
        {
            settings.controlMode = BallhandlerBoardControlMode::BALLHANDLER_CONTROL_MODE_OFF;
        }
        _piData.setBallhandlerSettings(settings);

        // Angle and Velocity Setpoints
        BallhandlerSetpoints setpoints;
        setpoints.angleLeft = ballHandlersMotorSetpoint.bhMotorData.angleLeft;
        setpoints.angleRight = ballHandlersMotorSetpoint.bhMotorData.angleRight;
        setpoints.velocityLeft = ballHandlersMotorSetpoint.bhMotorData.velocityLeft;
        setpoints.velocityRight = ballHandlersMotorSetpoint.bhMotorData.velocityRight;
        _piData.setBallhandlerSetpoints(setpoints);
    }

}
