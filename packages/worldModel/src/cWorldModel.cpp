// Copyright 2015-2020 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cWorldModel.cpp
 *
 *  Created on: January, 2019
 *      Author: Jan Feitsma
 */

#include "int/cWorldModel.hpp"
#include "tracing.hpp"


cWorldModel::cWorldModel()
    : _wmConfig(),
      _robotAdmin(_wmConfig),
      _ballAdmin(_wmConfig),
      _obstacleAdmin(_wmConfig)
{
    initialize();
}

void cWorldModel::initialize()
{
    // initialize adapters
    _adpCollector.initializeRTDB();
    
    // RTDB input adapter
    _rtdbInput.setBallAdministrator(&_ballAdmin);
    _rtdbInput.setObstacleAdministrator(&_obstacleAdmin);
    _rtdbInput.setRobotAdministrator(&_robotAdmin);
    _adpCollector.setRTDBInputAdapter(&_rtdbInput);
    
    // RTDB output adapter
    _rtdbOutput.setBallAdministrator(&_ballAdmin);
    _rtdbOutput.setObstacleAdministrator(&_obstacleAdmin);
    _rtdbOutput.setRobotAdministrator(&_robotAdmin);
    _adpCollector.setRTDBOutputAdapter(&_rtdbOutput);

    // add admins to adapters
    _adpCollector.setBallAdministrator(&_ballAdmin);
    _adpCollector.setObstacleAdministrator(&_obstacleAdmin);
    _adpCollector.setRobotAdministrator(&_robotAdmin);

    // attach adapters
    _adpHeartBeatCoach.setUpdateFunction(boost::bind(&cWorldModel::updateNow, this, _1)); // coach
    _adpHeartBeatRobot.setUpdateFunction(boost::bind(&cWorldModel::updateNow, this, _1)); // robots
}

cWorldModel::~cWorldModel()
{
}

void cWorldModel::run()
{
    // Block on the robot heartbeat (waitForPut)
    _adpHeartBeatRobot.run();
}

void cWorldModel::enableInplayOverrule()
{
    _rtdbInput.enableInplayOverrule();
}

void cWorldModel::updateNow(bool dummy)
{
    update(rtime::now());
}

void cWorldModel::update(rtime const timeNow)
{
    _adpCollector.heartBeatRecalculation(timeNow);
    WRITE_TRACE;
}

diagWorldModel cWorldModel::getDiagnostics()
{
    return _adpCollector.getDiagnostics();
}

