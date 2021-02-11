// Copyright 2019-2020 Erik Kouters (Falcons)
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
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId, teamChar);

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
        std::vector<std::string> params;
        switch(actionData.action)
        {
            case actionTypeEnum::MOVE:
            {
                _motionPlanner->setAction(MP_ActionMoveToTarget());
                params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
                params.push_back(boost::lexical_cast<std::string>((int)actionData.motionType));
                params.push_back(boost::lexical_cast<std::string>((int)actionData.ballHandlersEnabled));
                break;
            }
            case actionTypeEnum::KICK:
            {
                _motionPlanner->setAction(MP_ActionKick());
                params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
                break;
            }
            case actionTypeEnum::PASS:
            {
                _motionPlanner->setAction(MP_ActionPassToTarget());
                params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
                break;
            }
            case actionTypeEnum::SHOOT:
            {
                _motionPlanner->setAction(MP_ActionShootAtTarget());
                params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.z));
                params.push_back("SHOOT");
                break;
            }
            case actionTypeEnum::LOB:
            {
                _motionPlanner->setAction(MP_ActionShootAtTarget());
                params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.z));
                params.push_back("LOB");
                break;
            }
            case actionTypeEnum::STOP:
            {
                _motionPlanner->setAction(MP_ActionStop());
                params.push_back(boost::lexical_cast<std::string>((int)actionData.ballHandlersEnabled));
                break;
            }
            case actionTypeEnum::GET_BALL:
            {
                _motionPlanner->setAction(MP_ActionGetBall());
                params.push_back(boost::lexical_cast<std::string>((int)actionData.motionType));
                break;
            }
            case actionTypeEnum::TURN_AWAY_FROM_OPPONENT:
            {
                _motionPlanner->setAction(MP_ActionTurnAwayFromOpponent());
                params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
                params.push_back(boost::lexical_cast<std::string>((int)actionData.motionType));
                break;
            }
            case actionTypeEnum::KEEPER_MOVE:
            {
                _motionPlanner->setAction(MP_ActionKeeperMove());
                params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
                break;
            }
            case actionTypeEnum::INTERCEPT_BALL:
            {
                _motionPlanner->setAction(MP_ActionInterceptBall());
                params.push_back(boost::lexical_cast<std::string>((int)actionData.motionType));
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
