// Copyright 2018-2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * gameData.cpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Coen Tempelaars
 */

#include "int/gameData.hpp"

#include "tracing.hpp"

bool GameData::anyRobotHasBall() const
{
    bool result = false;

    for (const auto& teampair: team)
    {
        for (const auto& robotpair: teampair.second)
        {
            const auto& robot = robotpair.second;
            if (robot.hasBall())
            {
                result = true;
            }
        }
    }

    TRACE((result) ? ("A robot has the ball") : ("No robot has the ball"));
    return result;
}

bool GameData::anyRobotIsMoving() const
{
    bool result = false;

    for (const auto& teampair: team)
    {
        for (const auto& robotpair: teampair.second)
        {
            const auto& robot = robotpair.second;
            if (robot.isMoving())
            {
                result = true;
            }
        }
    }

    TRACE((result) ? ("A robot is moving") : ("No robot is moving"));
    return result;
}

float GameData::getDistanceOfClosestRobotTo(const Point2D& p, const TeamID teamID) const
{
    float result = std::numeric_limits<float>::max();

    for (const auto& robotpair: team.at(teamID))
    {
        const auto& robot = robotpair.second;
        if (robot.getDistanceTo(p) < result)
        {
            result = robot.getDistanceTo(p);
        }
    }

    return result;
}

boost::optional<TeamID> GameData::getTeamWithBall() const
{
    boost::optional<TeamID> result = boost::none;

    for (const auto& teampair: team)
    {
        for (const auto& robotpair: teampair.second)
        {
            const auto& robot = robotpair.second;
            if (robot.hasBall())
            {
                result = teampair.first;
            }
        }
    }

    TRACE(result ? ("A team has the ball") : ("No team has the ball"));
    return result;
}

boost::optional<RobotID> GameData::getRobotWithBall(const TeamID teamID) const
{
    boost::optional<RobotID> result = boost::none;

    for (const auto& robotpair: team.at(teamID))
    {
        const auto& robot = robotpair.second;
        if (robot.hasBall())
        {
            result = robotpair.first;
        }
    }

    TRACE(result ? ("A robot has the ball") : ("No robot has the ball"));
    return result;
}
