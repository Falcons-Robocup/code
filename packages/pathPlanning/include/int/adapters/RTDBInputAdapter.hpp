// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBInputAdapter.hpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#ifndef RTDBINPUTADAPTER_PATHPLANNING_HPP_
#define RTDBINPUTADAPTER_PATHPLANNING_HPP_

#include "int/InputInterface.hpp"
#include "cWorldModelClient.hpp"
#include "FalconsRtDB2.hpp"


class RTDBInputAdapter : public InputInterface
{
public:
    RTDBInputAdapter();
    ~RTDBInputAdapter();

    void fetch();
    void waitForMotionSetpoint();

    motionSetpoint              getMotionSetpoint();
    std::vector<forbiddenArea>  getForbiddenAreas();
    robotState                  getRobotState();
    std::vector<robotState>     getTeamMembers();
    std::vector<ballResult>     getBalls();
    std::vector<obstacleResult> getObstacles();

private:
    cWorldModelClient* _wmClient;
    RtDB2 *_rtdb;
    int _myRobotId;

    motionSetpoint              _motionSetpoint;
    std::vector<forbiddenArea>  _forbiddenAreas;
    robotState                  _robot;
    std::vector<robotState>     _teamMembers;
    std::vector<ballResult>     _balls;
    std::vector<obstacleResult> _obstacles;

};

#endif

