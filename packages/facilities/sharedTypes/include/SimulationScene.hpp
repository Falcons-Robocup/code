// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * SimulationScene.hpp
 *
 *  Created on: November, 2019
 *      Author: Jan Feitsma
 *
 */

#ifndef SIMULATIONSCENE_HPP_
#define SIMULATIONSCENE_HPP_

#include "pose.hpp"
#include "vec2d.hpp"
#include "vec3d.hpp"


// custom data types w.r.t. the regular ones from worldModel (robotState, ballResult etc), because:
// * pretty serialization (no bare list), readability in rtop and .scene files
// * remove clutter which is to be calculated, focus purely on the things to be manipulated


struct SimulationSceneRobot
{
    int  robotId;
    pose position;
    pose velocity;
    SERIALIZE_DATA(robotId, position, velocity);
};

struct SimulationSceneBall
{
    vec3d position;
    vec3d velocity;
    SERIALIZE_DATA(position, velocity);
};

struct SimulationSceneObstacle
{
    vec2d position;
    vec2d velocity;
    SERIALIZE_DATA(position, velocity);
};

struct SimulationScene
{
    std::vector<SimulationSceneRobot>     robots; // team A
    std::vector<SimulationSceneRobot>     opponents; // team B
    std::vector<SimulationSceneBall>      balls; // 0 or 1
    std::vector<SimulationSceneObstacle>  obstacles; // extra w.r.t. teamB

    SERIALIZE_DATA(robots, opponents, balls, obstacles);
};

#endif

