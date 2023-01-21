// Copyright 2018-2021 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * simworldGameData.hpp
 *
 *  Created on: Nov 21, 2018
 *      Author: Coen Tempelaars
 */

#ifndef SIMWORLDGAMEDATA_HPP_
#define SIMWORLDGAMEDATA_HPP_

#include "circle.hpp"
#include "gameData.hpp"
#include "teamID.hpp"
#include "vector2d.hpp"
#include "SimulationScene.hpp"

class SimworldGameData : public GameData {
public:
    SimworldGameData() : GameData() {}
    SimworldGameData(GameData const &g) : GameData(g) {}
    void recalculateWorld(const float dt);
    void setScene(SimulationScene const &scene);

private:
    void recalculateBall(const float dt);
    void recalculateRobot (Robot& robot, const float dt);
    void resolveBallToRobotCollision(const Circle& other, const Vector2D& otherspeed, const float dt);
    void resolveRobotToRobotCollision(Robot& robot, const Circle& other, const Vector2D& otherspeed, const float dt);
    std::vector<Obstacle> getBallObstacles() const;
};

#endif /* SIMWORLDGAMEDATA_HPP_ */
