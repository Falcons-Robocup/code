// Copyright 2018-2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * gameDataTest.cpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Coen Tempelaars
 */

#include <gtest/gtest.h>
#include "int/gameDataFactory.hpp"


class AGame : public ::testing::Test
{
public:
    AGame()
    {
        gameData = GameDataFactory::createGameData(5, 5);
    }

    GameData gameData;
};


TEST_F(AGame, ConsistsOfTwoTeams)
{
    EXPECT_EQ(2, gameData.team.size());
}

TEST_F(AGame, ConsistsOfFiveRobotsPerTeam)
{
    EXPECT_EQ(5, gameData.team[TeamID::A].size());
    EXPECT_EQ(5, gameData.team[TeamID::B].size());
}

TEST_F(AGame, NoRobotIsMoving)
{
    EXPECT_FALSE(gameData.anyRobotIsMoving());
}

TEST_F(AGame, OneRobotIsMoving)
{
    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(1.0, 1.0, 0.0));
    EXPECT_TRUE(gameData.anyRobotIsMoving());
}

TEST_F(AGame, DistanceOfClosestRobotToPOI)
{
    EXPECT_FLOAT_EQ(1.0, gameData.getDistanceOfClosestRobotTo(Point2D(0.0, -1.0), TeamID::A));
    EXPECT_FLOAT_EQ(3.0, gameData.getDistanceOfClosestRobotTo(Point2D(0.0, -1.0), TeamID::B));
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
