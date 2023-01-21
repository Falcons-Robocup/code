// Copyright 2019-2021 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * preparingController.cpp
 *
 *  Created on: Jan 8, 2019
 *      Author: Coen Tempelaars
 */

#include "int/preparingController.hpp"

#include "tracing.hpp"

const static float minimumWaitingTime =  1.0; //seconds
const static float maximumWaitingTime = 10.0; // seconds


boost::signals2::connection PreparingController::preparedSignalSubscribe (const signal_t::slot_type& subscriber)
{
    return _preparedSignal.connect(subscriber);
}

boost::signals2::connection PreparingController::stoppingSignalSubscribe (const signal_t::slot_type& subscriber)
{
    return _stoppingSignal.connect(subscriber);
}

void PreparingController::declareGamePrepared()
{
    if (!_signalSent)
    {
        // raise the 'prepared' signal
        _preparedSignal();

        // remember that a signal has been sent
        _signalSent = true;
    }
}

void PreparingController::declareGameStopping()
{
    if (!_signalSent)
    {
        // raise the 'stopping' signal
        _stoppingSignal();

        // remember that a signal has been sent
        _signalSent = true;
    }
}

void PreparingController::control (const ArbiterGameData& gamedata)
{
    control(gamedata, 0.0);
}

void PreparingController::control (const ArbiterGameData& gamedata,
                                   const float secondsSinceLastTransition)
{
    TRACE_FUNCTION("");
    _signalSent = false;

    if (gamedata.ball.isMoving())
    {
        TRACE("Ball is moving, stopping.");
        declareGameStopping();
    }

    if (secondsSinceLastTransition > minimumWaitingTime)
    {
        if (!gamedata.anyRobotIsMoving())
        {
            TRACE("No more robots moving and minimum waiting time finished, game prepared.");
            declareGamePrepared();
        }
    }

    if (secondsSinceLastTransition > maximumWaitingTime)
    {
        TRACE("Maximum waiting time exceeded, game prepared.");
        declareGamePrepared();
    }
}
