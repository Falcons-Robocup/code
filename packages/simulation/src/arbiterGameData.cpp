// Copyright 2018-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * arbiterGameData.cpp
 *
 *  Created on: Dec 29, 2018
 *      Author: Coen Tempelaars
 */

#include "int/arbiterGameData.hpp"

#include "cEnvironmentField.hpp"

const static float SCRUM_THRESHOLD__BALL_SPEED = 0.1;
const static float SCRUM_THRESHOLD__BALL_TO_ROBOT_DISTANCE = 1.0;

using boost::accumulators::rolling_mean;
using boost::accumulators::tag::rolling_window;


ArbiterGameData::ArbiterGameData()
: teamLastHoldingBall(TeamID::A)
, robotLastHoldingBall(RobotID::r1)
, ballSpeedAccumulator(rolling_window::window_size = 100)
, ballToRobotDistanceAccumulatorTeamA(rolling_window::window_size = 100)
, ballToRobotDistanceAccumulatorTeamB(rolling_window::window_size = 100)
{ }


void ArbiterGameData::refresh(const GameData& gameData)
{
    ball = gameData.ball;
    team = gameData.team;

    for (const auto& teampair: team)
    {
        for (const auto& robotpair: teampair.second)
        {
            const auto& robot = robotpair.second;
            if (robot.canKickBall(ball.getPosition()))
            {
                teamLastHoldingBall = teampair.first;
                robotLastHoldingBall = robotpair.first;
            }
        }
    }

    ballSpeedAccumulator(ball.getSpeed());

    if (team.find(TeamID::A) != team.end())
    {
        ballToRobotDistanceAccumulatorTeamA(getDistanceOfClosestRobotTo(ball.getLocation(), TeamID::A));
    }

    if (team.find(TeamID::B) != team.end())
    {
        ballToRobotDistanceAccumulatorTeamB(getDistanceOfClosestRobotTo(ball.getLocation(), TeamID::B));
    }
}


bool ArbiterGameData::ballIsInNegativeGoal() const
{
    const auto ballLocation = ball.getLocation();

    return  (  (ballLocation.y < -0.5 * cEnvironmentField::getInstance().getLength())
            && (abs(ballLocation.x) < cEnvironmentField::getInstance().getGoalPostOffset())
            );
}

bool ArbiterGameData::ballIsInPositiveGoal() const
{
    const auto ballLocation = ball.getLocation();

    return  (  (ballLocation.y > 0.5 * cEnvironmentField::getInstance().getLength())
            && (abs(ballLocation.x) < cEnvironmentField::getInstance().getGoalPostOffset())
            );
}

bool ArbiterGameData::ballIsAcrossNegativeGoalline() const
{
    const auto ballLocation = ball.getLocation();

    return  (  (ballLocation.y < -0.5 * cEnvironmentField::getInstance().getLength())
            &&  (abs(ballLocation.x) > cEnvironmentField::getInstance().getGoalPostOffset())
            );
}

bool ArbiterGameData::ballIsAcrossPositiveGoalline() const
{
    const auto ballLocation = ball.getLocation();

    return  (  (ballLocation.y > 0.5 * cEnvironmentField::getInstance().getLength())
            && (abs(ballLocation.x) > cEnvironmentField::getInstance().getGoalPostOffset())
            );
}

bool ArbiterGameData::ballIsAcrossSideLine() const
{
    return (abs(ball.getLocation().x) > (0.5 * cEnvironmentField::getInstance().getWidth()));
}

bool ArbiterGameData::isScrum() const
{
    return (  (rolling_mean(ballSpeedAccumulator) < SCRUM_THRESHOLD__BALL_SPEED)
           && (rolling_mean(ballToRobotDistanceAccumulatorTeamA) < SCRUM_THRESHOLD__BALL_TO_ROBOT_DISTANCE)
           && (rolling_mean(ballToRobotDistanceAccumulatorTeamB) < SCRUM_THRESHOLD__BALL_TO_ROBOT_DISTANCE)
    );
}
