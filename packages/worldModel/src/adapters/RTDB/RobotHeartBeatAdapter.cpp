// Copyright 2020-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RobotHeartBeatAdapter.cpp
 *
 *  Created on: Mar 20, 2019
 *      Author: Jan Feitsma
 */

#include "int/adapters/RTDB/RobotHeartBeatAdapter.hpp"

#include "falconsCommon.hpp"

RobotHeartBeatAdapter::RobotHeartBeatAdapter()
{
    _myRobotId = getRobotNumber();
    initializeRtDB();
}

RobotHeartBeatAdapter::~RobotHeartBeatAdapter()
// Chuck Norris is the reason Waldo is hiding
{
}

void RobotHeartBeatAdapter::initializeRtDB()
{
    _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(_myRobotId);
}

void RobotHeartBeatAdapter::run()
{
    while (true)
    {
        //_rtdb->waitForPut(BALL_CANDIDATES);
        _rtdb->waitForPut(HEARTBEAT_WORLDMODEL);
        notify(true);
    }
}

