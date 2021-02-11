// Copyright 2015-2020 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cWorldModel.hpp
 *
 *  Created on: January, 2019
 *      Author: Jan Feitsma
 */

#ifndef CWORLDMODEL_HPP_
#define CWORLDMODEL_HPP_


#include "int/administrators/ballAdministrator.hpp"
#include "int/administrators/obstacleAdministrator.hpp"
#include "int/administrators/robotAdministrator.hpp"
#include "int/adapters/adaptersCollector.hpp"
#include "int/adapters/RTDB/RobotHeartBeatAdapter.hpp"
#include "int/adapters/RTDB/CoachHeartBeatAdapter.hpp"

#include "int/adapters/RTDB/RTDBInputAdapter.hpp"

#include "ext/WorldModelNames.h"

#include "cDiagnostics.hpp"



class cWorldModel
{
public:
    cWorldModel();
    ~cWorldModel();
    void enableInplayOverrule();

    void run(); // block on waitForPut
    void updateNow(bool dummy); // use current timestamp
    void update(rtime const timeNow); // use provided timestamp

    diagWorldModel getDiagnostics();
    
protected:
    void initialize();

    // config
    WorldModelConfig         _wmConfig;
    
    // administrators
    robotAdministrator       _robotAdmin;
    ballAdministrator        _ballAdmin;
    obstacleAdministrator    _obstacleAdmin;

    // adapters
    adaptersCollector        _adpCollector;
    RTDBInputAdapter         _rtdbInput;
    RTDBOutputAdapter        _rtdbOutput;
    RobotHeartBeatAdapter    _adpHeartBeatRobot;
    CoachHeartBeatAdapter    _adpHeartBeatCoach;

};

#endif

