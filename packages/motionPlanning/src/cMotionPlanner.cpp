// Copyright 2017-2020 Jan Feitsma (Falcons)
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

bool cMotionPlanner::noAction()
{
    TRACE("> actionPtr=%p", _action);
    bool result = _action == NULL;
    TRACE("< result=%d", result);
    return result;
}

bool cMotionPlanner::checkActionEqual(std::string name)
{
    TRACE("> actionPtr=%p", _action);
    bool result = false;
    if (_action != NULL)
    {
        result = (typeid(*_action).name() == name);
    }
    TRACE("< result=%d", result);
    return result;
}

void cMotionPlanner::clearAction()
{
    TRACE("> actionPtr=%p", _action);
    if (_action != NULL)
    {
        delete _action;
    }
    TRACE("<");
}

void cMotionPlanner::initiateAction(MP_AbstractAction *action)
{
    TRACE("> actionPtr=%p", _action);
    _action = action;
    _action->connect(&_interfaces);
    TRACE("<");
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

    // clear if done
    if (result.result != actionResultTypeEnum::RUNNING)
    {
        TRACE("cleanup");
        delete _action;
        _action = NULL;
    }
    TRACE("< result=%s", enum2str(result.result));
    return result;
}

const cQueryInterface& cMotionPlanner::getQI()
{
    return _queryInterface;
}
