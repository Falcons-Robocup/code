// Copyright 2017-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cBallHandlingInterface.cpp
 *
 *  Created on: Dec 3, 2017
 *      Author: Jan Feitsma
 */

#include "int/cBallHandlingInterface.hpp"
#include "ftime.hpp"

void cBallHandlingInterface::tick()
{
    rtime timeNow = ftime::now(); // TODO this is not simulator- and test-friendly, better to move timestamping outside
    if (_isSuppressed && (double(timeNow) >= double(_enableTimestamp)))
    {
        enableBallHandlers();
        _isSuppressed = false;
    }
}

void cBallHandlingInterface::suppress(float timeout)
{
    disableBallHandlers();
    _isSuppressed = true;
    rtime timeNow = ftime::now(); // TODO this is not simulator- and test-friendly, better to move timestamping outside
    _enableTimestamp = timeNow + timeout;
}

