// Copyright 2019-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionTurnAwayFromOpponent.cpp
 *
 *  Created on: Apr 25, 2018
 *      Author: Erik Kouters
 */

#include "../include/int/MP_ActionTurnAwayFromOpponent.hpp"

#include "falconsCommon.hpp"
#include <stdexcept>
#include "cDiagnostics.hpp"


MP_ActionTurnAwayFromOpponent::MP_ActionTurnAwayFromOpponent()
{
}

void MP_ActionTurnAwayFromOpponent::initialize()
{
    TRACE_FUNCTION("");
}

actionResultTypeEnum MP_ActionTurnAwayFromOpponent::execute()
{
    TRACE_FUNCTION("");
    
    // check basics conditions to return FAILED (robot not inplay)
    if (!_wm->isActive())
    {
        return actionResultTypeEnum::FAILED;
    }

    // unpack parameters
    unpackParameters();

    // get current position and calculate deltas
    calculate();

    // move by writing setpoint to RTDB and calling pathPlanning iteration
    return setMotionSetpointAndCalculate(actionTypeEnum::MOVE, _targetPos, motionTypeEnum::NORMAL);
}

void MP_ActionTurnAwayFromOpponent::unpackParameters()
{
    _opponentPos.x = boost::lexical_cast<float>(_params.at(0));
    _opponentPos.y = boost::lexical_cast<float>(_params.at(1));
}

void MP_ActionTurnAwayFromOpponent::calculate()
{
    _currentPos = _wm->getPosition();
    _targetPos = _currentPos;

    float targetPhi = angle_between_two_points_0_2pi(_currentPos.x, _currentPos.y, _opponentPos.x, _opponentPos.y);
    targetPhi = project_angle_0_2pi(targetPhi + M_PI);

    _targetPos.phi = targetPhi;
}

