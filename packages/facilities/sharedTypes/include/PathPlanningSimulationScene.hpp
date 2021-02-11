// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * PathPlanningSimulationScene.hpp
 *
 *  Created on: November, 2019
 *      Author: Jan Feitsma
 *
 */

#ifndef PATHPLANNINGSIMULATIONSCENE_HPP_
#define PATHPLANNINGSIMULATIONSCENE_HPP_

#include "SimulationScene.hpp"
#include "wayPoint.hpp"
#include "forbiddenArea.hpp"


struct LocalizationNoise
{
    float xy = 0.0;
    float Rz = 0.0;
    SERIALIZE_DATA_FIXED(xy, Rz);
};

// extend standard SimulationScene with some pathPlanning specifics
struct PathPlanningSimulationScene: public SimulationScene
{
    int                                   robotId; // which robot to control
    bool                                  hasBall; // does robot have the ball
    wayPoint                              target; // target of the robot
    std::vector<forbiddenArea>            forbiddenAreas; // optional extra forbidden areas
    LocalizationNoise                     localizationNoise;

    SERIALIZE_DATA(robots, opponents, balls, obstacles, robotId, hasBall, target, forbiddenAreas, localizationNoise);
};

#endif

