// Copyright 2018-2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * gameData.hpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Coen Tempelaars
 */

#ifndef GAMEDATA_HPP_
#define GAMEDATA_HPP_

#include <map>
#include <vector>
#include "boost/optional.hpp"

#include "ball.hpp"
#include "obstacle.hpp"
#include "teamData.hpp"
#include "teamID.hpp"

class GameData {
public:
    bool anyRobotHasBall() const;
    bool anyRobotIsMoving() const;

    boost::optional<TeamID> getTeamWithBall() const;
    boost::optional<RobotID> getRobotWithBall(const TeamID) const;

    float getDistanceOfClosestRobotTo(const Point2D&, const TeamID) const;

    /* the following attributes are deliberately public */
    typedef std::map<TeamID, TeamData> Team;
    Team team;
    Ball ball;
    std::vector<Obstacle> nonRobotObstacles; // extra/artificial (handy for testing), visible to both teams
};

#endif /* GAMEDATA_HPP_ */
