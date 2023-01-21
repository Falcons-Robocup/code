// Copyright 2017-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cMotionPlanner.cpp
 *
 *  Created on: Nov 17, 2017
 *      Author: Jan Feitsma
 */

#include "int/cMotionPlanner.hpp"
#include "cDiagnostics.hpp"
#include "falconsCommon.hpp"
#include "int/adapters/configuration/cConfigMotionPlanningData.hpp"

using namespace std;

cConfigMotionPlanningData* _cfg;

cMotionPlanner::cMotionPlanner(MP_WorldModelInterface *wm, PathPlanningClient *pp, MP_RTDBOutputAdapter *rtdbOutput)
{
    //setAction(STOP);
    _interfaces.wm = wm;
    _interfaces.pp = pp;
    _interfaces.rtdbOutput = rtdbOutput;

    _cfg = new cConfigMotionPlanningData();

    _queryInterface.connect(wm);

    TRACE("cMotionPlanner constructed");
    //PTRACE("INITIALIZED");
}

cMotionPlanner::~cMotionPlanner()
{
    delete _cfg;
}

void cMotionPlanner::setAction(actionTypeEnum actionType)
{
    TRACE_FUNCTION("");
    // Only create a new action if:
    // - no action exists yet, or
    // - when a new action is chosen
    if (_action == NULL || _currentAction != actionType)
    {
        TRACE("Creating new action");
        // Delete the old action (if it exists)
        if (_action != NULL)
        {
            delete _action;
            TRACE_CONTEXT_FINISH(enum2str(_currentAction), _action);
            _action = 0;
        }

        switch(actionType)
        {
            case actionTypeEnum::MOVE:
            {
                _action = new MP_ActionMoveToTarget();
                break;
            }
            case actionTypeEnum::KICK:
            {
                _action = new MP_ActionKick();
                break;
            }
            case actionTypeEnum::PASS:
            {
                _action = new MP_ActionPassToTarget();
                break;
            }
            case actionTypeEnum::SHOOT:
            {
                _action = new MP_ActionShootAtTarget();
                break;
            }
            case actionTypeEnum::LOB:
            {
                _action = new MP_ActionShootAtTarget();
                break;
            }
            case actionTypeEnum::STOP:
            {
                _action = new MP_ActionStop();
                break;
            }
            case actionTypeEnum::GET_BALL:
            {
                _action = new MP_ActionGetBall();
                break;
            }
            case actionTypeEnum::TURN_AWAY_FROM_OPPONENT:
            {
                _action = new MP_ActionTurnAwayFromOpponent();
                break;
            }
            case actionTypeEnum::KEEPER_MOVE:
            {
                _action = new MP_ActionKeeperMove();
                break;
            }
            case actionTypeEnum::INTERCEPT_BALL:
            {
                _action = new MP_ActionInterceptBall();
                break;
            }
            default:
            {
                throw std::runtime_error("Unknown enum value received");
                break;
            }
        }

        TRACE_CONTEXT_START(enum2str(actionType), _action);

        // Connect the action to all data interfaces
        _action->connect(&_interfaces);
        _action->setConfig( _cfg->getConfiguration() );
        _action->initialize();

        _currentAction = actionType;
    }
}

void cMotionPlanner::setActionParameters(std::vector<std::string> const &params)
{
    TRACE("> actionPtr=%p", _action);
    // set the parameters of current action
    assert(_action != NULL);
    _action->setParameters(params);
    TRACE("<");
}

actionResult cMotionPlanner::execute()
{
    TRACE_FUNCTION("");

    TRACE("> actionPtr=%p", _action);
    // update worldModel
    _interfaces.wm->update();

    // update config
    _action->setConfig( _cfg->getConfiguration() );

    // execute current action
    assert(_action != NULL);
    actionResult result;
    result.result = _action->execute();

    // Write ACTION_RESULT to RTDB
    _interfaces.rtdbOutput->setActionResult(result);

    TRACE("< result=%s", enum2str(result.result));
    return result;
}

const cQueryInterface& cMotionPlanner::getQI()
{
    return _queryInterface;
}
