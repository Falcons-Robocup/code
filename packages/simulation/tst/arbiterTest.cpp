// Copyright 2019-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * arbiterTest.cpp
 *
 * Integration test for the arbiter class including its dependencies:
 * preparingController, runningController, stoppingController and stoppedController
 *
 *  Created on: Jan 10, 2019
 *      Author: Coen Tempelaars
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "int/arbiter.hpp"

const static float SIMULATION_PERIOD_FOR_TEST = 0.05;

class MockAbstractRefBoxAdapter : public AbstractRefBoxAdapter {
public:
    MOCK_CONST_METHOD0(republish, void());
    MOCK_METHOD0(sendStart, void());
    MOCK_METHOD0(sendStop, void());
    MOCK_METHOD2(sendSetpiece, void(const Setpiece&, const TeamID&));
    MOCK_METHOD1(registerGoal, void(const TeamID&));
};


class TestableArbiter : public Arbiter
{
public:
    TestableArbiter(AbstractRefBoxAdapter& refBoxAdapter)
    {
        _refBoxAdapter = &refBoxAdapter;
    }
};


class AnArbiter : public ::testing::Test
{
public:
    AnArbiter()
    : arbiter(mockAbstractRefBoxAdapter)
    {
        // The ball is somewhere in the field, but not at the center
        gameData.ball.setLocation(Point2D(1.5, 2.5));
    }

    MockAbstractRefBoxAdapter mockAbstractRefBoxAdapter;
    TestableArbiter arbiter;
    ArbiterGameData gameData;
};


class AnArbiterForARunningGame : public AnArbiter {
public:
    AnArbiterForARunningGame()
    : _gameState(GameState::STOPPED)
    {
        EXPECT_CALL(mockAbstractRefBoxAdapter, sendSetpiece(Setpiece::KICKOFF, TeamID::A));
        EXPECT_CALL(mockAbstractRefBoxAdapter, sendStart());

        /*
         * Let the arbiter control the game until the game is running
         */
        while (arbiter.getGameState() != GameState::RUNNING)
        {
            arbiter.control(gameData, SIMULATION_PERIOD_FOR_TEST);
        }
    }

    GameState _gameState;
};


TEST_F(AnArbiterForARunningGame, DoesNotStopTheGameWhileTheBallIsInside)
{
    EXPECT_CALL(mockAbstractRefBoxAdapter, sendStop()).Times(0);

    /* Hack: give the ball a big impulse to avoid a scrum (being followed by stop and dropball) */
    gameData.ball.setVelocity(Vector3D(3.0, 3.0, 0.0));

    gameData.ball.setLocation(Point2D(-1.5, 2.5));
    arbiter.control(gameData, SIMULATION_PERIOD_FOR_TEST);
    EXPECT_EQ(GameState::RUNNING, arbiter.getGameState());

    gameData.ball.setLocation(Point2D(-1.5, -2.5));
    arbiter.control(gameData, SIMULATION_PERIOD_FOR_TEST);
    EXPECT_EQ(GameState::RUNNING, arbiter.getGameState());

    gameData.ball.setLocation(Point2D(1.5, -2.5));
    arbiter.control(gameData, SIMULATION_PERIOD_FOR_TEST);
    EXPECT_EQ(GameState::RUNNING, arbiter.getGameState());

    gameData.ball.setLocation(Point2D(1.5, 2.5));
    arbiter.control(gameData, SIMULATION_PERIOD_FOR_TEST);
    EXPECT_EQ(GameState::RUNNING, arbiter.getGameState());
}


TEST_F(AnArbiterForARunningGame, ArrangesAThrowinIfTheBallIsAcrossTheSideline)
{
    EXPECT_CALL(mockAbstractRefBoxAdapter, sendStop());
    EXPECT_CALL(mockAbstractRefBoxAdapter, sendSetpiece(Setpiece::THROWIN, TeamID::B));

    gameData.ball.setLocation(Point2D(7.1, 3.0));
    gameData.teamLastHoldingBall = TeamID::A;

    while (arbiter.getGameState() != GameState::PREPARING)
    {
        arbiter.control(gameData, SIMULATION_PERIOD_FOR_TEST);
    }
}


TEST_F(AnArbiterForARunningGame, StopsTheGameIfTheBallIsOutside)
{
    EXPECT_CALL(mockAbstractRefBoxAdapter, sendStop());

    gameData.ball.setLocation(Point2D(20.0, 20.0));

    while (arbiter.getGameState() != GameState::STOPPED)
    {
        arbiter.control(gameData, SIMULATION_PERIOD_FOR_TEST);
    }
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
