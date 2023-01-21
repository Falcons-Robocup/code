// Copyright 2019-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RtdbAdapterControlOverride.cpp
 *
 *  Created on: April, 2019
 *      Author: Jan Feitsma
 */

#include "int/adapters/RtdbAdapterControlOverride.hpp"

#include "falconsCommon.hpp" //getRobotNumber()
#include "cDiagnostics.hpp"

RtdbAdapterControlOverride::RtdbAdapterControlOverride()
{
    _myRobotId = getRobotNumber();
    _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(_myRobotId, getTeamChar());
}

RtdbAdapterControlOverride::~RtdbAdapterControlOverride()
{
}

bool RtdbAdapterControlOverride::getOverrideState(T_TP_OVERRIDE_STATE &overrideState)
{
    int ageMs = 0;

    int r = _rtdb->get(TP_OVERRIDE_STATE, &overrideState, ageMs, _myRobotId);

    return (r == RTDB2_SUCCESS) && (ageMs < 200);
}

void RtdbAdapterControlOverride::setOverrideResult(T_TP_OVERRIDE_RESULT const &overrideResult)
{
    _rtdb->put(TP_OVERRIDE_RESULT, &overrideResult);
}

