// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBInputKeeperFrameAdapter.cpp
 *
 *  Created on: Mar 28, 2019
 *      Author: Jan Feitsma
 */

#include "int/ioBoard/RTDBInputKeeperFrameAdapter.hpp"

#include <exception>

#include "falconsCommon.hpp"
#include <cDiagnostics.hpp>
#include "tracing.hpp"

RTDBInputKeeperFrameAdapter::RTDBInputKeeperFrameAdapter(KeeperFrame& kf)
    : _keeperFrame(kf)
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(_myRobotId);
    TRACE("<");
}

RTDBInputKeeperFrameAdapter::~RTDBInputKeeperFrameAdapter()
{
    TRACE(">");
    TRACE("<");
}

void RTDBInputKeeperFrameAdapter::loop()
{
    while (true)
    {
        _rtdb->waitForPut(KEEPERFRAME_SETPOINT);
        process();
    }
}

void RTDBInputKeeperFrameAdapter::process()
{
    TRACE_FUNCTION("");
    T_KEEPERFRAME_SETPOINT keeperFrameSetpoint;
    int ageMs = 0, r = 0;

    r = _rtdb->get(KEEPERFRAME_SETPOINT, &keeperFrameSetpoint, ageMs, _myRobotId);
    if ((r == RTDB2_SUCCESS) && (ageMs < 100))
    {
        tprintf("received new keeperFrame setpoint: %s", enum2str(keeperFrameSetpoint));
        TRACE_INFO_TIMEOUT(1.0, "extending keeperFrame %s", enum2str(keeperFrameSetpoint));
        
        switch (keeperFrameSetpoint)
        {
            case keeperFrameSetpointEnum::NONE:
                break;
            case keeperFrameSetpointEnum::LEFT:
                _keeperFrame.setKeeperFrameLeft();
                break;
            case keeperFrameSetpointEnum::RIGHT:
                _keeperFrame.setKeeperFrameRight();
                break;
            case keeperFrameSetpointEnum::UP:
                _keeperFrame.setKeeperFrameUp();
                break;
            case keeperFrameSetpointEnum::UNKNOWN:
            default:
                TRACE_ERROR_TIMEOUT(1.0, "received invalid keeperFrame setpoint");
        }
    }
}

