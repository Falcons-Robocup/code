// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RobotHeartBeatAdapter.hpp
 *
 *  Created on: Mar 19, 2019
 *      Author: Jan Feitsma
 */

#ifndef ROBOTHEARTBEATADAPTER_HPP_
#define ROBOTHEARTBEATADAPTER_HPP_

#include "int/facilities/templatedSubject.hpp"
#include <boost/thread/thread.hpp>
#include "FalconsRTDB.hpp"

class RobotHeartBeatAdapter : public templatedSubject<bool>
{
public:
    RobotHeartBeatAdapter();
    ~RobotHeartBeatAdapter();
    void run();

private:
    void initializeRtDB();
    
    boost::thread _heartBeatThread;
    RtDB2 *_rtdb;
    int _myRobotId;

};

#endif /* ROBOTHEARTBEATADAPTER_HPP_ */
