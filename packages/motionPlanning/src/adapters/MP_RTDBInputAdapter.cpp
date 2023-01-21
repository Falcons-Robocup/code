// Copyright 2019-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cRTDBInputAdapter.cpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#include "../../include/int/adapters/MP_RTDBInputAdapter.hpp"
#include "falconsCommon.hpp" //getRobotNumber(), getTeamChar()
#include "cDiagnostics.hpp"
#include "tracing.hpp"

MP_RTDBInputAdapter::MP_RTDBInputAdapter(cMotionPlanner* mp)
{
    TRACE_FUNCTION("");
    _myRobotId = getRobotNumber();
    auto teamChar = getTeamChar();
    _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(_myRobotId, teamChar);

    _motionPlanner = mp;
}

MP_RTDBInputAdapter::~MP_RTDBInputAdapter()
{
}

void MP_RTDBInputAdapter::waitForActionData()
{
    while (true)
    {
        int result = _rtdb->waitForPut(K_ACTION);
        if (result == RTDB2_SUCCESS)
        {
            getActionData();
        }

        WRITE_TRACE;
    }
}

void MP_RTDBInputAdapter::getActionData()
{
    TRACE_FUNCTION("");
    T_ACTION actionData;

    int r = _rtdb->get(K_ACTION, &actionData);

    tprintf("get ACTION %s position=[%6.2f, %6.2f, %6.2f] motionType=%s bh=%d",
        enum2str(actionData.action), actionData.position.x, actionData.position.y, actionData.position.z, enum2str(actionData.motionType), actionData.ballHandlersEnabled);

    if (r == RTDB2_SUCCESS)
    {
        // Set the action.
        // If the same action is already active, the old action is kept alive and reused (to maintain state).
        // Otherwise, a new action is created.
        _motionPlanner->setAction(actionData.action);

        // For each action, retrieve the parameters and store for use when executing the action
        std::vector<std::string> params;
        switch(actionData.action)
        {
            case actionTypeEnum::MOVE:
            {
                params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.z)); // Rz actually
                params.push_back(boost::lexical_cast<std::string>((int)actionData.motionType));
                params.push_back(boost::lexical_cast<std::string>((int)actionData.ballHandlersEnabled));
                break;
            }
            case actionTypeEnum::KICK:
            {
                params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
                break;
            }
            case actionTypeEnum::PASS:
            {
                params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
                break;
            }
            case actionTypeEnum::SHOOT:
            {
                params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.z));
                params.push_back("SHOOT");
                break;
            }
            case actionTypeEnum::LOB:
            {
                params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.z));
                params.push_back("LOB");
                break;
            }
            case actionTypeEnum::STOP:
            {
                params.push_back(boost::lexical_cast<std::string>((int)actionData.ballHandlersEnabled));
                break;
            }
            case actionTypeEnum::GET_BALL:
            {
                params.push_back(boost::lexical_cast<std::string>((int)actionData.motionType));
                break;
            }
            case actionTypeEnum::TURN_AWAY_FROM_OPPONENT:
            {
                params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
                break;
            }
            case actionTypeEnum::KEEPER_MOVE:
            {
                break;
            }
            case actionTypeEnum::INTERCEPT_BALL:
            {
                break;
            }
            default:
            {
                throw std::runtime_error("Unknown enum value received from RTDB ActionData");
                break;
            }
        }
        _motionPlanner->setActionParameters(params);
        _motionPlanner->execute();
    }
}
