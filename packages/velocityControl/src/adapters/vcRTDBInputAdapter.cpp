// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * vcRTDBInputAdapter.cpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#include "int/adapters/vcRTDBInputAdapter.hpp"

#include "falconsCommon.hpp" //getRobotNumber(), getTeamChar()
#include "cDiagnostics.hpp"
#include "tracing.hpp"

vcRTDBInputAdapter::vcRTDBInputAdapter()
{
    TRACE_FUNCTION("");
    _myRobotId = getRobotNumber();
    auto teamChar = getTeamChar();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId, teamChar);
    _wmClient = new cWorldModelClient();
}

vcRTDBInputAdapter::~vcRTDBInputAdapter()
{
    TRACE_FUNCTION("");
}

void vcRTDBInputAdapter::waitForRobotPosVelSetpoint()
{
    TRACE_FUNCTION("");
    _rtdb->waitForPut(ROBOT_POSVEL_SETPOINT);
}

T_ROBOT_POSVEL_SETPOINT vcRTDBInputAdapter::getRobotPosVelSetpoint()
{
    return _robotPosVelSetpoint;
}

robotState vcRTDBInputAdapter::getRobotState()
{
    return _robot;
}

std::vector<robotState> vcRTDBInputAdapter::getTeamMembers()
{
    return _teamMembers;
}

std::vector<obstacleResult> vcRTDBInputAdapter::getObstacles()
{
    return _obstacles;
}

std::vector<ballResult> vcRTDBInputAdapter::getBalls()
{
    return _balls;
}

void vcRTDBInputAdapter::fetch()
{
    TRACE_FUNCTION("");

    int r = 0;

    // robotPosVelSetpoint
    r = _rtdb->get(ROBOT_POSVEL_SETPOINT, &_robotPosVelSetpoint);
    if (r == RTDB2_SUCCESS)
    {
        tprintf("get ROBOT_POSVEL_SETPOINT posVelEnum=%s pos=[%6.2f, %6.2f, %6.2f] vel=[%6.2f, %6.2f, %6.2f] motionType=%s", enum2str(_robotPosVelSetpoint.robotPosVelType), _robotPosVelSetpoint.position.x, _robotPosVelSetpoint.position.y, _robotPosVelSetpoint.position.Rz, _robotPosVelSetpoint.velocity.x, _robotPosVelSetpoint.velocity.y, _robotPosVelSetpoint.velocity.Rz, enum2str(_robotPosVelSetpoint.motionType));
    }
    else
    {
        TRACE_ERROR("failed to get ROBOT_POSVEL_SETPOINT");
    }

    // poke worldModel to update
    _wmClient->update();

    // store own robot position and velocity
    r = _wmClient->getRobotState(_robot, _myRobotId);
    if (r == false)
    {
        TRACE_ERROR("failed to get robotState from worldModel");
    }

    // get obstacles, balls and teammembers
    _obstacles = _wmClient->getObstacles(); // opponents actually
    _balls = _wmClient->getBalls();
    _teamMembers = _wmClient->getTeamMembersExcludingSelf();

}

