// Copyright 2019-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cAbstractAction.hpp
 *
 *  Created on: Nov 17, 2017
 *      Author: Jan Feitsma
 */

#ifndef MP_ABSTRACTACTION_HPP_
#define MP_ABSTRACTACTION_HPP_

#include <vector>
#include <string>
#include <boost/lexical_cast.hpp>

#include "adapters/MP_RTDBOutputAdapter.hpp"

#include "int/cInterfaces.hpp"
#include "int/cTimer.hpp"
#include "tracing.hpp"
#include "FalconsRtDB2.hpp" // actionResult

class MP_AbstractAction
{
public:
    MP_AbstractAction();
    virtual ~MP_AbstractAction();
    
    void setParameters(std::vector<std::string> const &params);
    virtual actionResultTypeEnum execute() = 0;
    void connect(cInterfaces *interfaces = NULL);
    float elapsed(); // time in seconds
    
    // shared convenience functions
    void stopMoving();
    actionResultTypeEnum setMotionSetpointAndCalculate(actionTypeEnum action, Position2D const &target, motionTypeEnum motionType, bool autostop = true);
    ConfigMotionPlanning getConfig();
    void setConfig(ConfigMotionPlanning config);

protected:
    std::vector<std::string> _params;
    bool _isConnected;
    ConfigMotionPlanning _config;
    
    MP_WorldModelInterface *_wm;
    PathPlanningClient *_pp;
    MP_RTDBOutputAdapter *_rtdbOutput;

    cTimer _actionTimer;
};

#endif /* MP_ABSTRACTACTION_HPP_ */

