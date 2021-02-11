// Copyright 2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * stoppingControllerTest.cpp
 *
 *  Created on: Jan 7, 2019
 *      Author: Coen Tempelaars
 */

#include <gtest/gtest.h>

#include "int/stoppingController.hpp"


class AStoppingController : public ::testing::Test
{
public:
    AStoppingController()
    : gameIsStopped(false)
    {
        stoppingController.stoppedSignalSubscribe(boost::bind(&AStoppingController::stoppedSignalHandler, this));
    }

    void stoppedSignalHandler()
    {
        gameIsStopped = true;
    }

    StoppingController stoppingController;
    ArbiterGameData gameData;
    bool gameIsStopped;
};


TEST_F(AStoppingController, StopsTheGameWhenNoRobotMoves)
{
    // Two robots are moving
    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(1.0, 1.0, 0.0));
    gameData.team[TeamID::A][RobotID::r2].setVelocity(Velocity2D(1.0, 1.0, 0.0));

    stoppingController.control(gameData);
    EXPECT_FALSE(gameIsStopped);

    // One robot stops moving
    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(0.0, 0.0, 0.0));

    stoppingController.control(gameData);
    EXPECT_FALSE(gameIsStopped);

    // The last moving robot stops moving
    gameData.team[TeamID::A][RobotID::r2].setVelocity(Velocity2D(0.0, 0.0, 0.0));

    stoppingController.control(gameData);
    EXPECT_TRUE(gameIsStopped);
}


TEST_F(AStoppingController, StopsTheGameWhenTimeElapses)
{
    // Two robots are moving
    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(1.0, 1.0, 0.0));
    gameData.team[TeamID::A][RobotID::r2].setVelocity(Velocity2D(1.0, 1.0, 0.0));

    stoppingController.control(gameData);
    EXPECT_FALSE(gameIsStopped);

    // One robot stops moving
    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(0.0, 0.0, 0.0));

    stoppingController.control(gameData);
    EXPECT_FALSE(gameIsStopped);

    // Time almost elapses
    stoppingController.control(gameData, 4.99);
    EXPECT_FALSE(gameIsStopped);

    // Time elapses
    stoppingController.control(gameData, 5.01);
    EXPECT_TRUE(gameIsStopped);
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
