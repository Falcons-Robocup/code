// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * vcRTDBInputAdapter.hpp
 *
 *  Created on: Oct, 2020
 *      Author: Erik Kouters
 */

#ifndef RTDBINPUTADAPTER_VELOCITYCONTROL_HPP_
#define RTDBINPUTADAPTER_VELOCITYCONTROL_HPP_

#include "int/InputInterface.hpp"
#include "cWorldModelClient.hpp"
#include "FalconsRtDB2.hpp"


class vcRTDBInputAdapter : public InputInterface
{
public:
    vcRTDBInputAdapter();
    ~vcRTDBInputAdapter();

    void fetch();
    void waitForRobotPosVelSetpoint();

    robotPosVel                 getRobotPosVelSetpoint();
    robotState                  getRobotState();
    std::vector<robotState>     getTeamMembers();
    std::vector<ballResult>     getBalls();
    std::vector<obstacleResult> getObstacles();

private:
    cWorldModelClient* _wmClient;
    RtDB2 *_rtdb;
    int _myRobotId;

    robotPosVel                 _robotPosVelSetpoint;
    robotState                  _robot;
    std::vector<robotState>     _teamMembers;
    std::vector<ballResult>     _balls;
    std::vector<obstacleResult> _obstacles;

};

#endif

