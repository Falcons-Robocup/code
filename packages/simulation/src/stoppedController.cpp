// Copyright 2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * stoppedController.cpp
 *
 *  Created on: Jan 9, 2019
 *      Author: Coen Tempelaars
 */

#include "int/stoppedController.hpp"

#include "tracing.hpp"


boost::signals2::connection StoppedController::preparingSignalSubscribe (const signal_t::slot_type& subscriber)
{
    return _preparingSignal.connect(subscriber);
}

void StoppedController::declareGamePreparing()
{
    if (!_signalSent)
    {
        // raise the 'preparing' signal
        _preparingSignal();

        // remember that a signal has been sent
        _signalSent = true;
    }
}

void StoppedController::control (const ArbiterGameData& gameData)
{
    TRACE_FUNCTION("");
    _signalSent = false;
    declareGamePreparing();
}
