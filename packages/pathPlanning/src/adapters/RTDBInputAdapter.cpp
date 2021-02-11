// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBInputAdapter.cpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#include "int/adapters/RTDBInputAdapter.hpp"

#include "falconsCommon.hpp" //getRobotNumber(), getTeamChar()
#include "cDiagnostics.hpp"
#include "tracing.hpp"

RTDBInputAdapter::RTDBInputAdapter()
{
    TRACE_FUNCTION("");
    _myRobotId = getRobotNumber();
    auto teamChar = getTeamChar();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId, teamChar);
    _wmClient = new cWorldModelClient();
}

RTDBInputAdapter::~RTDBInputAdapter()
{
    TRACE_FUNCTION("");
}

void RTDBInputAdapter::waitForMotionSetpoint()
{
    TRACE_FUNCTION("");
    _rtdb->waitForPut(MOTION_SETPOINT);
}

T_MOTION_SETPOINT RTDBInputAdapter::getMotionSetpoint()
{
    return _motionSetpoint;
}

std::vector<forbiddenArea> RTDBInputAdapter::getForbiddenAreas()
{
    return _forbiddenAreas;
}

robotState RTDBInputAdapter::getRobotState()
{
    return _robot;
}

std::vector<robotState> RTDBInputAdapter::getTeamMembers()
{
    return _teamMembers;
}

std::vector<obstacleResult> RTDBInputAdapter::getObstacles()
{
    return _obstacles;
}

std::vector<ballResult> RTDBInputAdapter::getBalls()
{
    return _balls;
}

void RTDBInputAdapter::fetch()
{
    TRACE_FUNCTION("");

    int r = 0;

    // motionSetpoint
    r = _rtdb->get(MOTION_SETPOINT, &_motionSetpoint);
    if (r == RTDB2_SUCCESS)
    {
        tprintf("get MOTION_SETPOINT action=%s pos=[%6.2f, %6.2f, %6.2f] motionType=%s", enum2str(_motionSetpoint.action), _motionSetpoint.position.x, _motionSetpoint.position.y, _motionSetpoint.position.z, enum2str(_motionSetpoint.motionType));
    }
    else
    {
        TRACE_ERROR("failed to get MOTION_SETPOINT");
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

    // get external forbidden areas
    T_FORBIDDEN_AREAS tmpForbiddenAreas;
    r = _rtdb->get(FORBIDDEN_AREAS, &tmpForbiddenAreas); // it may happen that this data is not filled (yet)
    if (r == RTDB2_SUCCESS)
    {
        _forbiddenAreas = tmpForbiddenAreas;
        TRACE("fetched %d forbidden areas", (int)tmpForbiddenAreas.size());
    }
    else
    {
        TRACE("clearing forbidden areas");
        _forbiddenAreas.clear();
    }
}

