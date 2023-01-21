// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBInputKickerAdapter.cpp
 *
 *  Created on: Sep 23, 2018
 *      Author: Jan Feitsma
 */

#include "int/ioBoard/cRTDBInputKickerAdapter.hpp"

#include <exception>

#include "falconsCommon.hpp"
#include <cDiagnostics.hpp>
#include "tracing.hpp"

cRTDBInputKickerAdapter::cRTDBInputKickerAdapter(Kicker& kicker)
    : kicker(kicker)
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(_myRobotId);
    TRACE("<");
}

cRTDBInputKickerAdapter::~cRTDBInputKickerAdapter()
{
    TRACE(">");
    TRACE("<");
}

void cRTDBInputKickerAdapter::waitForKickerSetpoint()
{
    while (true)
    {
        _rtdb->waitForPut(KICKER_SETPOINT);
        getKickerSetpoint();
    }
}

void cRTDBInputKickerAdapter::getKickerSetpoint()
{
    TRACE_FUNCTION("");
    T_KICKER_SETPOINT kickerSetpoint;

    _rtdb->get(KICKER_SETPOINT, &kickerSetpoint);

    switch(kickerSetpoint.kickerSetpointType)
    {
        case kickerSetpointTypeEnum::HOME:
        {
            TRACE_SCOPE("HOME_KICKER","");
            kicker.home();
            break;
        }
        case kickerSetpointTypeEnum::SET_HEIGHT:
        {
            std::stringstream str;
            str << "height=" << kickerSetpoint.kickerHeight;
            TRACE_SCOPE("SET_KICKER_HEIGHT", str.str().c_str());

            kicker.move(kickerSetpoint.kickerHeight);
            break;
        }
        case kickerSetpointTypeEnum::SHOOT:
        {
            std::stringstream str;
            str << "power=" << kickerSetpoint.kickerPower;
            TRACE_SCOPE("SET_KICKER_POWER", str.str().c_str());

            // only shoot if power > 0
            if (kickerSetpoint.kickerPower > 0)
            {
                kicker.shoot(kickerSetpoint.kickerPower);
            }
            break;
        }
    }

}
