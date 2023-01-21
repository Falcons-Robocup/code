// Copyright 2019-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionMoveToTarget.cpp
 *
 *  Created on: Nov 26, 2017
 *      Author: Jan Feitsma
 */

#include "../include/int/MP_ActionMoveToTarget.hpp"

#include "falconsCommon.hpp"
#include <stdexcept>
#include "cDiagnostics.hpp"


MP_ActionMoveToTarget::MP_ActionMoveToTarget()
{
    _result = actionResultTypeEnum::RUNNING;
}

void MP_ActionMoveToTarget::initialize()
{
    TRACE_FUNCTION("");
}

actionResultTypeEnum MP_ActionMoveToTarget::execute()
{
    TRACE_FUNCTION("");

    // check basics conditions to return FAILED (robot not inplay)
    if (!_wm->isActive())
    {
        TRACE("WM inactive -- robot is probably out of play");
        return actionResultTypeEnum::FAILED;
    }

    // unpack parameters
    unpackParameters();

    // poke lower components
    _rtdbOutput->setBallHandlersSetpoint(_ballHandlersEnabled);

    // move by writing setpoint to RTDB and calling pathPlanning iteration
    _result = setMotionSetpointAndCalculate(actionTypeEnum::MOVE, _targetPos, _motionType);

    // wrap up
    return _result;
}

void MP_ActionMoveToTarget::unpackParameters()
{
    _targetPos.x = boost::lexical_cast<float>(_params.at(0));
    _targetPos.y = boost::lexical_cast<float>(_params.at(1));
    _targetPos.phi = boost::lexical_cast<float>(_params.at(2));
    _motionType = (motionTypeEnum)(boost::lexical_cast<int>(_params.at(3)));
    _ballHandlersEnabled = boost::lexical_cast<int>(_params.at(4));
}

