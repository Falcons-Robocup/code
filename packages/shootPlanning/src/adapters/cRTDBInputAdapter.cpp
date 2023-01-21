// Copyright 2019-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBInputAdapter.cpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Erik Kouters
 */

#include "tracing.hpp"
#include "falconsCommon.hpp" //getRobotNumber(), getTeamChar()

#include "int/adapters/cRTDBInputAdapter.hpp"

cRTDBInputAdapter::cRTDBInputAdapter(cShootPlanner *shootPlanner)
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    auto teamChar = getTeamChar();
    tprintf("cRTDBInputAdapter get RTDB start");
    _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(_myRobotId, teamChar);
    tprintf("cRTDBInputAdapter get RTDB end");
    _shootPlanner = shootPlanner;
    TRACE("<");
}

cRTDBInputAdapter::~cRTDBInputAdapter()
{
}

void cRTDBInputAdapter::waitForShootSetpoint()
{
    while (true)
    {
        _rtdb->waitForPut(SHOOT_SETPOINT);
        getShootSetpoint();
    }
}

void cRTDBInputAdapter::getShootSetpoint()
{
    TRACE_FUNCTION("");
    T_SHOOT_SETPOINT shootSetpoint;

    int r = _rtdb->get(SHOOT_SETPOINT, &shootSetpoint);
    
    tprintf("             get SHOOT_SETPOINT phase=%s type=%s pos=[%6.2f, %6.2f, %6.2f]", enum2str(shootSetpoint.shootPhase), enum2str(shootSetpoint.shootType), shootSetpoint.position.x, shootSetpoint.position.y, shootSetpoint.position.z);

    if (r == RTDB2_SUCCESS)
    {
        switch(shootSetpoint.shootPhase)
        {
            case shootPhaseEnum::NONE:
            {
                // do nothing
                break;
            }
            case shootPhaseEnum::PREPARE:
            {
                if (_shootPlanner != NULL)
                {
                    _shootPlanner->prepareForShot(shootSetpoint.position.x, shootSetpoint.position.z, shootSetpoint.shootType);
                }
                break;
            }
            case shootPhaseEnum::SHOOT:
            {
                switch(shootSetpoint.shootType)
                {
                    case shootTypeEnum::PASS:
                    {
                        if (_shootPlanner != NULL)
                        {
                            _shootPlanner->executePass(shootSetpoint.position.x);
                        }
                        break;
                    }
                    case shootTypeEnum::SHOOT:
                    {
                        if (_shootPlanner != NULL)
                        {
                            _shootPlanner->executeShot();
                        }
                        break;
                    }
                    case shootTypeEnum::LOB:
                    {
                        if (_shootPlanner != NULL)
                        {
                            _shootPlanner->executeShot();
                        }
                        break;
                    }
                }
                break;
            }
        }
    }
}
