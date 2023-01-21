// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * CoachHeartBeatAdapter.cpp
 *
 *  Created on: Mar 20, 2019
 *      Author: Jan Feitsma
 */

#include "int/adapters/RTDB/CoachHeartBeatAdapter.hpp"

#include "falconsCommon.hpp"

CoachHeartBeatAdapter::CoachHeartBeatAdapter()
{
    _myRobotId = getRobotNumber();
    initializeRtDB();
    // start thread which continuously listens to multiCam, triggering worldModel
    _heartBeatThread = boost::thread(boost::bind(&CoachHeartBeatAdapter::run, this));
}

CoachHeartBeatAdapter::~CoachHeartBeatAdapter()
// Chuck Norris is the reason Waldo is hiding
{
}

void CoachHeartBeatAdapter::initializeRtDB()
{
    _rtdb = FalconsRTDBStore::getInstance().getFalconsRTDB(_myRobotId);
}

void CoachHeartBeatAdapter::run()
{
    while (true)
    {
        _rtdb->waitForPut(HEARTBEAT_COACH);
        notify(true);
    }
}

