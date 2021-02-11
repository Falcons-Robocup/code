// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * CoachHeartBeatAdapter.hpp
 *
 *  Created on: Mar 19, 2019
 *      Author: Jan Feitsma
 */

#ifndef COACHHEARTBEATADAPTER_HPP_
#define COACHHEARTBEATADAPTER_HPP_

#include "int/facilities/templatedSubject.hpp"
#include <boost/thread/thread.hpp>
#include "FalconsRtDB2.hpp"

class CoachHeartBeatAdapter : public templatedSubject<bool>
{
public:
    CoachHeartBeatAdapter();
    ~CoachHeartBeatAdapter();

private:
    void initializeRtDB();
    void run();
    
    boost::thread _heartBeatThread;
    RtDB2 *_rtdb;
    int _myRobotId;

};

#endif /* COACHHEARTBEATADAPTER_HPP_ */
