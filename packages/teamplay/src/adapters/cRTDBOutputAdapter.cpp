// Copyright 2018-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBOutputAdapter.cpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#include "int/adapters/cRTDBOutputAdapter.hpp"

#include "falconsCommon.hpp" //getRobotNumber(), getTeamChar()
#include "tracing.hpp"

cRTDBOutputAdapter::cRTDBOutputAdapter()
{
    TRACE_FUNCTION("");
    _myRobotId = getRobotNumber();
    auto teamChar = getTeamChar();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId, teamChar);
}

cMotionPlanningClient& cRTDBOutputAdapter::getMPClient()
{
    return _mpClient;
}

void cRTDBOutputAdapter::setActionData(const T_ACTION& actionData)
{
    TRACE_FUNCTION("");

    tprintf("put ACTION motionType=%s bh=%d position=[%6.2f, %6.2f, %6.2f] type=%s", enum2str(actionData.motionType), actionData.ballHandlersEnabled, actionData.position.x, actionData.position.y, actionData.position.z, enum2str(actionData.action));
    _rtdb->put(K_ACTION, &actionData);  
}

void cRTDBOutputAdapter::clearForbiddenAreas()
{
    TRACE_FUNCTION("");
    T_FORBIDDEN_AREAS forbiddenAreas; // empty array
    _rtdb->put(FORBIDDEN_AREAS, &forbiddenAreas);
}

void cRTDBOutputAdapter::setForbiddenAreas(const T_FORBIDDEN_AREAS& forbiddenAreas)
{
    TRACE_FUNCTION("");
    // enumerate id's
    auto tmpForbiddenAreas = forbiddenAreas;
    for (int it = 0; it < (int)tmpForbiddenAreas.size(); ++it)
    {
        tmpForbiddenAreas[it].id = it;
    }
    _rtdb->put(FORBIDDEN_AREAS, &tmpForbiddenAreas);
}

void cRTDBOutputAdapter::setRole(const T_ROBOT_ROLE& role)
{
    TRACE_FUNCTION("");

    _rtdb->put(ROBOT_ROLE, &role);
}

void cRTDBOutputAdapter::setIntention(const T_INTENTION& intention)
{
    TRACE_FUNCTION("");

    _rtdb->put(INTENTION, &intention);
}
