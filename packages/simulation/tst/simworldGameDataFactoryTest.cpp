// Copyright 2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * simworldGameDataFactoryTest.cpp
 *
 *  Created on: March 26, 2019
 *      Author: Coen Tempelaars
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "int/simworldGameDataFactory.hpp"


class TwoTeamsOfTwoRobots : public ::testing::Test
{
public:
    TwoTeamsOfTwoRobots()
    {
        gameData.ball.setLocation(Point2D(1.2, 3.4));
        gameData.team[TeamID::A][RobotID::r1].setPosition(Position2D(1.0, 1.0, 0.0));
        gameData.team[TeamID::A][RobotID::r2].setPosition(Position2D(2.0, 3.0, 0.0));
        gameData.team[TeamID::B][RobotID::r1].setPosition(Position2D(3.0, 4.0, 0.0));
        gameData.team[TeamID::B][RobotID::r2].setPosition(Position2D(4.0, 5.0, 0.0));
    }

    GameData gameData;
};


TEST_F(TwoTeamsOfTwoRobots, TransformedIntoCompleteWorld)
{
    auto simworldGameData = SimworldGameDataFactory::createCompleteWorld(gameData);
    EXPECT_EQ(2, simworldGameData.team.size());
    EXPECT_EQ(2, simworldGameData.team[TeamID::A].size());
    EXPECT_EQ(2, simworldGameData.team[TeamID::B].size());

    EXPECT_NEAR(2.0, gameData.team[TeamID::A][RobotID::r2].getPosition().x, 0.01);
    EXPECT_NEAR(5.0, gameData.team[TeamID::B][RobotID::r2].getPosition().y, 0.01);
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
