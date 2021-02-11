// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBOutputAdapter.cpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Erik Kouters
 */

#include "tracing.hpp"

#include "int/adapters/cRTDBOutputAdapter.hpp"

cRTDBOutputAdapter::cRTDBOutputAdapter()
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId);
    TRACE("<");
}

cRTDBOutputAdapter::~cRTDBOutputAdapter()
{
}

void cRTDBOutputAdapter::setRobotDisplacementFeedback(const vt_robot_data& robotData)
{
    std::stringstream str;
    str << "x=" << robotData.displacement.x << "; y=" << robotData.displacement.y << "; Rz=" << project_angle_mpi_pi(robotData.displacement.phi);
    TRACE_FUNCTION(str.str().c_str());

    // Put the latest displacement to RTDB

    T_ROBOT_DISPLACEMENT_FEEDBACK robotDisplacementFeedback;

    robotDisplacementFeedback.x = robotData.displacement.x;
    robotDisplacementFeedback.y = robotData.displacement.y;
    robotDisplacementFeedback.Rz = project_angle_mpi_pi(robotData.displacement.phi);
    _rtdb->put(ROBOT_DISPLACEMENT_FEEDBACK, &robotDisplacementFeedback);
}

void cRTDBOutputAdapter::setRobotVelocityFeedback(const vt_robot_data& robotData)
{
    std::stringstream str;
    str << "x=" << robotData.velocity.x << "; y=" << robotData.velocity.y << "; Rz=" << project_angle_mpi_pi(robotData.velocity.phi);
    TRACE_FUNCTION(str.str().c_str());

    // Get the current list of velocities from RTDB
    // Add the latest velocity to the list
    // Put the list back to RTDB

    T_ROBOT_VELOCITY_FEEDBACK robotVelocityFeedback;

    robotVelocityFeedback.x = robotData.velocity.x;
    robotVelocityFeedback.y = robotData.velocity.y;
    robotVelocityFeedback.Rz = robotData.velocity.phi;

    _rtdb->put(ROBOT_VELOCITY_FEEDBACK, &robotVelocityFeedback);
}

void cRTDBOutputAdapter::setMotorVelocitySetpoint(const vt_motors_data& motorsData)
{
    std::stringstream str;
    str << "m1=" << motorsData.m1.velocity << "; m2=" << motorsData.m2.velocity << "; m3=" << motorsData.m3.velocity;
    TRACE_FUNCTION(str.str().c_str());

    T_MOTOR_VELOCITY_SETPOINT motorVelocitySetpoint;
    motorVelocitySetpoint.m1 = motorsData.m1.velocity;
    motorVelocitySetpoint.m2 = motorsData.m2.velocity;
    motorVelocitySetpoint.m3 = motorsData.m3.velocity;

    _rtdb->put(MOTOR_VELOCITY_SETPOINT, &motorVelocitySetpoint);
}
