// Copyright 2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * preparingControllerTest.cpp
 *
 *  Created on: Jan 8, 2019
 *      Author: Coen Tempelaars
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "int/preparingController.hpp"


class APreparingController : public ::testing::Test
{
public:
    APreparingController()
    : gameIsPrepared(false)
    , gameIsStopping(false)
    {
        preparingController.preparedSignalSubscribe(boost::bind(&APreparingController::preparedSignalHandler, this));
        preparingController.stoppingSignalSubscribe(boost::bind(&APreparingController::stoppingSignalHandler, this));
    }

    void preparedSignalHandler()
    {
        gameIsPrepared = true;
    }

    void stoppingSignalHandler()
    {
        gameIsStopping = true;
    }

    PreparingController preparingController;
    ArbiterGameData gameData;
    bool gameIsPrepared;
    bool gameIsStopping;
};


TEST_F(APreparingController, DeclaresTheGamePreparedWhenNoRobotMoves)
{
    // Initially no robot moves, but that does not mean the game is prepared
    preparingController.control(gameData);
    EXPECT_FALSE(gameIsPrepared);

    // Two robots are moving
    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(1.0, 1.0, 0.0));
    gameData.team[TeamID::A][RobotID::r2].setVelocity(Velocity2D(1.0, 1.0, 0.0));

    preparingController.control(gameData);
    EXPECT_FALSE(gameIsPrepared);

    // After one second, one robot stops moving
    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(0.0, 0.0, 0.0));
    preparingController.control(gameData, 1.0);
    EXPECT_FALSE(gameIsPrepared);

    // After another second, the last moving robot stops moving
    gameData.team[TeamID::A][RobotID::r2].setVelocity(Velocity2D(0.0, 0.0, 0.0));
    preparingController.control(gameData, 2.0);
    EXPECT_TRUE(gameIsPrepared);
    EXPECT_FALSE(gameIsStopping);
}


TEST_F(APreparingController, DeclaresTheGamePreparedWhenTimeElapses)
{
    // Two robots are moving
    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(1.0, 1.0, 0.0));
    gameData.team[TeamID::A][RobotID::r2].setVelocity(Velocity2D(1.0, 1.0, 0.0));

    preparingController.control(gameData);
    EXPECT_FALSE(gameIsPrepared);

    // One robot stops moving
    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(0.0, 0.0, 0.0));

    preparingController.control(gameData);
    EXPECT_FALSE(gameIsPrepared);

    // Time almost elapses
    preparingController.control(gameData, 9.99);
    EXPECT_FALSE(gameIsPrepared);

    // Time elapses
    preparingController.control(gameData, 10.01);
    EXPECT_TRUE(gameIsPrepared);
    EXPECT_FALSE(gameIsStopping);
}


TEST_F(APreparingController, StopsTheGameWhenTheBallMoves)
{
    // Two robots are moving, but the ball is not
    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(1.0, 1.0, 0.0));
    gameData.team[TeamID::A][RobotID::r2].setVelocity(Velocity2D(1.0, 1.0, 0.0));
    gameData.ball.setVelocity(Vector3D());

    preparingController.control(gameData);
    EXPECT_FALSE(gameIsPrepared);
    EXPECT_FALSE(gameIsStopping);

    // The ball starts moving slightly
    gameData.ball.setVelocity(Vector3D(0.1, 0.1, 0.0));

    preparingController.control(gameData);
    EXPECT_FALSE(gameIsPrepared);
    EXPECT_TRUE(gameIsStopping);
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
