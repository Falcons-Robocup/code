// Copyright 2018-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBOutputAdapter.hpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#ifndef RTDBOUTPUTADAPTER_HPP_
#define RTDBOUTPUTADAPTER_HPP_

#include <vector>

#include "FalconsRTDB.hpp"

// WM's internal data administrators
#include "int/administrators/ballAdministrator.hpp"
#include "int/administrators/obstacleAdministrator.hpp"
#include "int/administrators/robotAdministrator.hpp"


class RTDBOutputAdapter
{
public:
    RTDBOutputAdapter();
    ~RTDBOutputAdapter();

    virtual void setBallAdministrator(ballAdministrator *ballAdmin);
    virtual void setObstacleAdministrator(obstacleAdministrator *obstacleAdmin);
    virtual void setRobotAdministrator(robotAdministrator *robotAdmin);

    void setRobotState();
    void setBalls(std::vector<ballClass_t> const &balls);
    void setObstacles();
    void setBallPossession(ballPossessionTypeEnum bpType, int bpRobot);

    // Reconfiguration
    void updateConfig(T_CONFIG_WORLDMODELSYNC const &config);

private:
    RtDB2 *_rtdb;
    int _myRobotId;
    ballPossessionTypeEnum _ballPossessionType;
    int _ballPossessionRobot = 0;
    T_CONFIG_WORLDMODELSYNC _config;

    // WM's internal data administrators
    ballAdministrator *_ballAdmin;
    obstacleAdministrator *_obstacleAdmin;
    robotAdministrator *_robotAdmin;

};

#endif

