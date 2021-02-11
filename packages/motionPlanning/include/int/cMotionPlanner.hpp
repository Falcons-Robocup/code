// Copyright 2017-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cMotionPlanner.hpp
 *
 *  Created on: Nov 17, 2017
 *      Author: Jan Feitsma
 */

#ifndef CMOTIONPLANNER_HPP_
#define CMOTIONPLANNER_HPP_

#include "falconsCommon.hpp"
#include "PathPlanningClient.hpp"
#include "int/cAllActions.hpp"
#include "int/cQueryInterface.hpp"


class cMotionPlanner
{
public:
    cMotionPlanner(MP_WorldModelInterface *wm, PathPlanningClient *pp, MP_RTDBOutputAdapter *rtdbOutput);
    ~cMotionPlanner();
    
    // set a the action (either continue current action or start a new one)
    template<typename T>
    void setAction(T const &action)
    {
        TRACE("setAction(%s)", typeid(action).name());
        if (noAction())
        {
            TRACE("first action");
            initiateAction((MP_AbstractAction *)(new T()));
        }
        else if (!checkActionEqual(typeid(action).name()))
        {
            TRACE("changing action");
            clearAction();
            initiateAction((MP_AbstractAction *)(new T()));
        }
        // else do nothing: running action remains alive
    }
    
    // set the parameters of current action
    void setActionParameters(std::vector<std::string> const &params);
    
    // execute current action
    actionResult execute();
    
    // interface getters (needed for relaying forbidden areas & suppressing ballhandlers)
    MP_RTDBOutputAdapter *getRTDBOutput();
    const cQueryInterface& getQI();

private:
    // current action and modifiers
    MP_AbstractAction *_action = NULL;
    bool noAction();
    bool checkActionEqual(std::string name);
    void clearAction();
    void initiateAction(MP_AbstractAction * action);
    
    // interfaces
    cInterfaces _interfaces;
    cQueryInterface _queryInterface;
    
};

#endif /* CMOTIONPLANNER_HPP_ */

