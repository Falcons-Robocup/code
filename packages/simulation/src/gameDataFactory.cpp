// Copyright 2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * gameDataFactory.cpp
 *
 *  Created on: March 4, 2019
 *      Author: Coen Tempelaars
 */

#include "int/gameDataFactory.hpp"

GameData GameDataFactory::createGameData(const int sizeTeamA, const int sizeTeamB)
{
    GameData gameData;

    /* Construct two teams of five robots each */
    gameData.team =
    {
        {TeamID::A,
            {
                {RobotID::r1, Robot()},
                {RobotID::r2, Robot()},
                {RobotID::r3, Robot()},
                {RobotID::r4, Robot()},
                {RobotID::r5, Robot()}
            }
        },
        {TeamID::B,
            {
                {RobotID::r1, Robot()},
                {RobotID::r2, Robot()},
                {RobotID::r3, Robot()},
                {RobotID::r4, Robot()},
                {RobotID::r5, Robot()}
            }
        }
    };

    /* Remove superfluous robots */
    auto it = gameData.team[TeamID::A].begin();
    for (int i = 0; i < sizeTeamA; i++) { it++; }
    gameData.team[TeamID::A].erase(it, gameData.team[TeamID::A].end());

    /* Set robot properties: location, playing direction, ... */
    double x = -4.0;
    for (auto& robotpair: gameData.team[TeamID::A])
    {
        const auto& robotID = robotpair.first;
        auto& robot = robotpair.second;
        robot.setPlayingDirection(PlayingDirection::LEFT_TO_RIGHT);
        robot.setPosition(Position2D(x, -2.0, 0.5 * M_PI));
        x += 2.0;

        if (robotID == RobotID::r1)
        {
            robot.setBallHandlingModuleAbsent();
        }
        else
        {
            robot.setBallHandlingModulePresent();
        }
    }

    it = gameData.team[TeamID::B].begin();
    for (int i = 0; i < sizeTeamB; i++) { it++; }
    gameData.team[TeamID::B].erase(it, gameData.team[TeamID::B].end());

    x = -4.0;
    for (auto& robotpair: gameData.team[TeamID::B])
    {
        const auto& robotID = robotpair.first;
        auto& robot = robotpair.second;
        robot.setPlayingDirection(PlayingDirection::RIGHT_TO_LEFT);
        robot.setPosition(Position2D(x,  2.0, 1.5 * M_PI));
        x += 2.0;

        if (robotID == RobotID::r1)
        {
            robot.setBallHandlingModuleAbsent();
        }
        else
        {
            robot.setBallHandlingModulePresent();
        }
    }

    return gameData;
}
