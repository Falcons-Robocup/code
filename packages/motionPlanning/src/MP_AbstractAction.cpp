// Copyright 2019-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cAbstractAction.cpp
 *
 *  Created on: Nov 17, 2017
 *      Author: Jan Feitsma
 */

#include "../include/int/MP_AbstractAction.hpp"

#include "falconsCommon.hpp"
#include "cDiagnostics.hpp"

using namespace std;

MP_AbstractAction::MP_AbstractAction()
{
    TRACE("construct");
    _isConnected = false;
    _actionTimer.reset();
}

MP_AbstractAction::~MP_AbstractAction()
{
    TRACE("delete");
}

void MP_AbstractAction::setParameters(std::vector<std::string> const &params)
{
    _params = params;
}

void MP_AbstractAction::connect(cInterfaces *interfaces)
{
    TRACE("connect");
    // store interfaces
    if (interfaces != NULL)
    {
        _wm = interfaces->wm;
        _pp = interfaces->pp;
        _rtdbOutput = interfaces->rtdbOutput;
        _isConnected = true;
    }
}

float MP_AbstractAction::elapsed()
{
    return _actionTimer.elapsed();
}

void MP_AbstractAction::stopMoving()
{
    setMotionSetpointAndCalculate(actionTypeEnum::STOP, Position2D(), motionTypeEnum::NORMAL, false);
}

actionResultTypeEnum MP_AbstractAction::setMotionSetpointAndCalculate(actionTypeEnum action, Position2D const &target, motionTypeEnum motionType, bool autostop)
{
    if (_rtdbOutput == NULL)
    {
        TRACE_ERROR("null pointer exception, _rtdbOutput not initialized");
        return actionResultTypeEnum::FAILED;
    }
    if (_pp == NULL)
    {
        TRACE_ERROR("null pointer exception, _pp not initialized");
        return actionResultTypeEnum::FAILED;
    }

    // write motion setpoint to RTDB
    _rtdbOutput->setMotionSetpoint(action, target, motionType);

    // poke pathPlanning
    auto result = _pp->iterate();

    // make sure to end with zero setpoint, to prevent drifting based on last nonzero
    // setpoint in combination with peripheralsInterface watchdog
    if (result != actionResultTypeEnum::RUNNING && autostop == true)
    {
        _rtdbOutput->setMotionSetpoint(actionTypeEnum::STOP, Position2D(), motionTypeEnum::NORMAL);
        (void)_pp->iterate(); // ignore result
    }
    return result;
}

ConfigMotionPlanning MP_AbstractAction::getConfig()
{
    return _config;
}

void MP_AbstractAction::setConfig(ConfigMotionPlanning config)
{
    _config = config;
}
