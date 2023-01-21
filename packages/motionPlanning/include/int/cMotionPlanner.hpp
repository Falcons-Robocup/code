// Copyright 2017-2021 Jan Feitsma (Falcons)
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
    
    void setAction(actionTypeEnum actionType);
    
    // set the parameters of current action
    void setActionParameters(std::vector<std::string> const &params);
    
    // execute current action
    actionResult execute();
    
    // interface getters (needed for relaying forbidden areas & suppressing ballhandlers)
    MP_RTDBOutputAdapter *getRTDBOutput();
    const cQueryInterface& getQI();

private:
    // current action and modifiers;
    actionTypeEnum _currentAction = actionTypeEnum::UNKNOWN;
    MP_AbstractAction *_action = NULL;
    
    // interfaces
    cInterfaces _interfaces;
    cQueryInterface _queryInterface;
    
};

#endif /* CMOTIONPLANNER_HPP_ */

