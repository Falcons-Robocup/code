// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBInputAdapter.cpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Erik Kouters
 */

#include "tracing.hpp"
#include "cDiagnostics.hpp"

#include "int/adapters/cRTDBInputAdapter.hpp"

cRTDBInputAdapter::cRTDBInputAdapter(cVelocityTransformData *data, iterateFunctionType feedbackfunc, iterateFunctionType setpointfunc)
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId);
    _vtData = data;
    _iterateFeedbackFunc = feedbackfunc;
    _iterateSetpointFunc = setpointfunc;
    TRACE("<");
}

cRTDBInputAdapter::~cRTDBInputAdapter()
{
}

void cRTDBInputAdapter::waitForMotorFeedback()
{
    while (true)
    {
        try
        {
            _rtdb->waitForPut(MOTOR_FEEDBACK);

            getMotorFeedback();

            _iterateFeedbackFunc();
        }
        catch(std::exception &e)
        {
            TRACE_ERROR("cRTDBInputAdapter::waitForMotorFeedback() failed: %s", e.what());
            std::cout << "cRTDBInputAdapter::waitForMotorFeedback() failed: " << e.what() << std::endl;
        }
    }
}

void cRTDBInputAdapter::waitForRobotVelocitySetpoint()
{
    while (true)
    {
        try
        {
            _rtdb->waitForPut(ROBOT_VELOCITY_SETPOINT);

            getRobotVelocitySetpoint();

            _iterateSetpointFunc();
        }
        catch(std::exception &e)
        {
            TRACE_ERROR("cRTDBInputAdapter::waitForRobotVelocitySetpoint() failed: %s", e.what());
            std::cout << "cRTDBInputAdapter::waitForRobotVelocitySetpoint() failed: " << e.what() << std::endl;
        }
    }
}

void cRTDBInputAdapter::getRobotVelocitySetpoint()
{
    TRACE_FUNCTION("");
    T_ROBOT_VELOCITY_SETPOINT robotVelSetpoint;

    int r = _rtdb->get(ROBOT_VELOCITY_SETPOINT, &robotVelSetpoint);

    if (r == RTDB2_SUCCESS)
    {
        tprintf("get ROBOT_VELOCITY_SETPOINT [%6.2f, %6.2f, %6.2f]", robotVelSetpoint.x, robotVelSetpoint.y, robotVelSetpoint.Rz);
        vt_robot_data robotData;
        robotData.velocity.x = robotVelSetpoint.x;
        robotData.velocity.y = robotVelSetpoint.y;
        robotData.velocity.phi = robotVelSetpoint.Rz;
        _vtData->setTargetRobotData(robotData);
    }
}

void cRTDBInputAdapter::getMotorFeedback()
{
    TRACE_FUNCTION("");
    T_MOTOR_FEEDBACK motorFeedback;

    int r = _rtdb->get(MOTOR_FEEDBACK, &motorFeedback);

    if (r == RTDB2_SUCCESS)
    {
        tprintf("get MOTOR_FEEDBACK disp[%6.2f, %6.2f, %6.2f] vel[%6.2f, %6.2f, %6.2f]", motorFeedback.m1.displacement, motorFeedback.m2.displacement, motorFeedback.m3.displacement, motorFeedback.m1.velocity, motorFeedback.m2.velocity, motorFeedback.m3.velocity);
        vt_motors_data motorData;
        motorData.m1.displacement = motorFeedback.m1.displacement;
        motorData.m2.displacement = motorFeedback.m2.displacement;
        motorData.m3.displacement = motorFeedback.m3.displacement;
        motorData.m1.velocity = motorFeedback.m1.velocity;
        motorData.m2.velocity = motorFeedback.m2.velocity;
        motorData.m3.velocity = motorFeedback.m3.velocity;
        _vtData->setFeedbackMotorsData(motorData);
    }

}
