// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * tpRTDBOutputAdapter.cpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#include "int/adapters/RTDBOutputAdapter.hpp"

#include "falconsCommon.hpp" //getRobotNumber(), getTeamChar()
#include "tracing.hpp"

tpRTDBOutputAdapter::tpRTDBOutputAdapter()
{
    TRACE_FUNCTION("");
    _myRobotId = getRobotNumber();
    auto teamChar = getTeamChar();
    _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(_myRobotId, teamChar);
}

void tpRTDBOutputAdapter::setActionData(const T_ACTION& actionData) const
{
    TRACE_FUNCTION("");

    tprintf("put ACTION motionType=%s bh=%d position=[%6.2f, %6.2f, %6.2f] type=%s", enum2str(actionData.motionType), actionData.ballHandlersEnabled, actionData.position.x, actionData.position.y, actionData.position.z, enum2str(actionData.action));
    _rtdb->put(K_ACTION, &actionData);  
}

void tpRTDBOutputAdapter::clearForbiddenAreas() const
{
    TRACE_FUNCTION("");
    T_FORBIDDEN_AREAS forbiddenAreas; // empty array
    _rtdb->put(FORBIDDEN_AREAS, &forbiddenAreas);
}

void tpRTDBOutputAdapter::setForbiddenAreas(const T_FORBIDDEN_AREAS& forbiddenAreas) const
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

void tpRTDBOutputAdapter::setRoles(const T_ROBOT_ROLES& roles) const
{
    TRACE_FUNCTION("");

    _rtdb->put(ROBOT_ROLES, &roles);
}

void tpRTDBOutputAdapter::setIntention(const T_INTENTION& intention) const
{
    TRACE_FUNCTION("");

    _rtdb->put(INTENTION, &intention);
}

void tpRTDBOutputAdapter::setLastUsedHeightmap(const T_HEIGHTMAP& heightmap) const
{
    TRACE_FUNCTION("");

    _rtdb->put(TP_HEIGHTMAP, &heightmap);
}