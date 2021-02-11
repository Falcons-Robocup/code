// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * KeeperFrame.cpp
 *
 *  Created on: Mar 26, 2019
 *      Author: Jan Feitsma
 */

#include <falconsCommon.hpp>
#include "cDiagnostics.hpp"

#include "int/ioBoard/KeeperFrame.hpp"

KeeperFrame::KeeperFrame(IoBoard &ioBoard) :
    ioBoard(ioBoard)
{
    _lastSetpointTime = rtime::now();
}

KeeperFrame::~KeeperFrame() {

}

void KeeperFrame::setKeeperFrameLeft()
{
    if (allowNewSetpoint())
    {
        TRACE_INFO("keeper frame extend LEFT");
        tprintf("setKeeperFrameLeft");
        ioBoard.setKeeperFrameLeft();
        _lastSetpointTime = rtime::now();
    }
}

void KeeperFrame::setKeeperFrameRight()
{
    if (allowNewSetpoint())
    {
        TRACE_INFO("keeper frame extend RIGHT");
        tprintf("setKeeperFrameRight");
        ioBoard.setKeeperFrameRight();
        _lastSetpointTime = rtime::now();
    }
}

void KeeperFrame::setKeeperFrameUp()
{
    if (allowNewSetpoint())
    {
        TRACE_INFO("keeper frame extend UP");
        tprintf("setKeeperFrameUp");
        ioBoard.setKeeperFrameUp();
        _lastSetpointTime = rtime::now();
    }
}

bool KeeperFrame::allowNewSetpoint()
{
    double elapsed = rtime::now() - _lastSetpointTime;
    return elapsed > KEEPERFRAME_SETPOINT_PERIOD;
}

