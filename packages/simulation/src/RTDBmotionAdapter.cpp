// Copyright 2019-2021 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBMotionAdapter.cpp
 *
 *  Created on: Feb 12, 2019
 *      Author: Coen Tempelaars
 */

#include "int/RTDBmotionAdapter.hpp"

#include "int/RTDBaccess.hpp"
#include "int/simulationCapabilities.hpp"

#include <stdexcept>

#include "FalconsRTDB.hpp"
#include "tracing.hpp"


Velocity2D RTDBMotionAdapter::getVelocity (const TeamID& teamID, const RobotID& robotID) const
{
    TRACE_FUNCTION("");
    T_ROBOT_VELOCITY_SETPOINT robotVelocitySetpoint;
    auto robotNumber = getRobotNumber(robotID);
    auto rtdbConnection = getRTDBConnection(teamID, robotID);
    auto r = rtdbConnection->get(ROBOT_VELOCITY_SETPOINT, &robotVelocitySetpoint, robotNumber);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error getting ROBOT_VELOCITY_SETPOINT from RtDB");
    }
    tprintf("get ROBOT_VELOCITY_SETPOINT [%d %d] robotVelocity=[%6.2f, %6.2f, %6.2f]",
                      int(teamID), int(robotID),
                      robotVelocitySetpoint.x,
                      robotVelocitySetpoint.y,
                      robotVelocitySetpoint.Rz);
    return Velocity2D(robotVelocitySetpoint.x,
                      robotVelocitySetpoint.y,
                      robotVelocitySetpoint.Rz);
}


Kicker RTDBMotionAdapter::getKickerData (const TeamID& teamID, const RobotID& robotID) const
{
    TRACE_FUNCTION("");
    T_KICKER_SETPOINT kickerSetpoint;
    auto robotNumber = getRobotNumber(robotID);
    auto rtdbConnection = getRTDBConnection(teamID, robotID);
    auto r = rtdbConnection->get(KICKER_SETPOINT, &kickerSetpoint, robotNumber);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error getting KICKER_SETPOINT from RtDB");
    }
    tprintf("get KICKER_SETPOINT [%d %d] kickerHeight=%6.2f kickerPower=%6.2f",
            int(teamID), int(robotID),
            kickerSetpoint.kickerHeight, kickerSetpoint.kickerPower);
    return {kickerSetpoint.kickerHeight, kickerSetpoint.kickerPower};
}


bool RTDBMotionAdapter::hasBallHandlersEnabled (const TeamID& teamID, const RobotID& robotID) const
{
    TRACE_FUNCTION("");
    T_BALLHANDLERS_SETPOINT ballhandlersSetpoint;
    auto robotNumber = getRobotNumber(robotID);
    auto rtdbConnection = getRTDBConnection(teamID, robotID);
    auto r = rtdbConnection->get(BALLHANDLERS_SETPOINT, &ballhandlersSetpoint, robotNumber);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error getting BALLHANDLERS_SETPOINT from RtDB");
    }
    tprintf("get BALLHANDLERS_SETPOINT [%d %d] enabled=%s",
            int(teamID), int(robotID),
            ((ballhandlersSetpoint)?("Yes"):("No ")));
    return ballhandlersSetpoint;
}
