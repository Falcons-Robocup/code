// Copyright 2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * stoppingController.cpp
 *
 *  Created on: Jan 7, 2019
 *      Author: Coen Tempelaars
 */

#include "int/stoppingController.hpp"

#include "tracing.hpp"

const static float maximumWaitingTime = 5.0; // seconds


boost::signals2::connection StoppingController::stoppedSignalSubscribe (const signal_t::slot_type& subscriber)
{
    return _stoppedSignal.connect(subscriber);
}

void StoppingController::declareGameStopped()
{
    if (!_signalSent)
    {
        // raise the 'stopped' signal
        _stoppedSignal();

        // remember that a signal has been sent
        _signalSent = true;
    }
}

void StoppingController::control (const ArbiterGameData& gamedata)
{
    control(gamedata, 0.0);
}

void StoppingController::control (const ArbiterGameData& gamedata,
                                  const float secondsSinceLastTransition)
{
    TRACE_FUNCTION("");
    _signalSent = false;

    if (!gamedata.anyRobotIsMoving())
    {
        declareGameStopped();
    }

    if (secondsSinceLastTransition > maximumWaitingTime)
    {
        declareGameStopped();
    }
}
