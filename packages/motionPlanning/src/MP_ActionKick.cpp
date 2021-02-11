// Copyright 2019-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionKick.cpp
 *
 *  Created on: April 2019
 *      Author: Jan Feitsma
 */

#include "../include/int/MP_ActionKick.hpp"

#include "cDiagnostics.hpp"


MP_ActionKick::MP_ActionKick()
{
    _actuatedKickerHeight = false;
    _actuatedBhDisable = false;
    // consecutive timers
    _heightTimer.setDuration(1.0); // TODO make reconfigurable
    // a bit long, but we have no feedback nor any guarantee how fast the height lever will move ... 
    // since this action is mostly intended for calibration purposes, better safe than sorry!
    _bhTimer.setDuration(getConfig().shootAtTargetConfig.disableBhDelay); // shared with pass- and shoot action
}

void MP_ActionKick::unpackParameters()
{
    _kickPower = boost::lexical_cast<float>(_params.at(0));
    _kickHeight = boost::lexical_cast<float>(_params.at(1));
}

actionResultTypeEnum MP_ActionKick::execute()
{
    unpackParameters();
    
    // kick is a special action for testing purposes
    // from robotCLI, it often happens that wm is not active or that we do not have the ball
    // so unlike in the 'production' shoot- and pass actions, here we do not check on wm->isActive.
    // We do check on wm->hasBall.
    // Exception: When the kick power is below *30*, we do allow a shot without ball.

    float powerThreshold = 30.0;
    if (!_wm->hasBall() && _kickPower > powerThreshold)
    {
        TRACE_WARNING("cannot kick without having the ball, unless power is below %.1f", powerThreshold);
        return actionResultTypeEnum::FAILED;
    }

    // done?
    if (_actuatedBhDisable && _bhTimer.expired())
    {
        // finish: shoot
        _rtdbOutput->setKickerPower(_kickPower);
        // TODO: wait until ball gone, then reset kicker height to zero and enable ballhandlers
        // maybe we need to instantiate a finite state machine to reduce code complexity, find common part for kick/pass/shoot
        return actionResultTypeEnum::PASSED;
    }
    // timed sequence
    if (!_actuatedKickerHeight)
    {
        // set kicker height once, only at the very start
        _rtdbOutput->setKickerHeight(_kickHeight);
        _actuatedKickerHeight = true;
    }
    // disable ballHandlers
    if (_heightTimer.expired())
    {
        if (!_actuatedBhDisable)
        {
            _bhTimer.reset();
            //_rtdbOutput->setBallHandlersSetpoint(false); // TODO: disabled in shoot so also here ... should merge implementations
            _actuatedBhDisable = true;
        }
    }
    return actionResultTypeEnum::RUNNING;
}

