 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
            arbiter.control(gameData);
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
    arbiter.control(gameData);
    EXPECT_EQ(GameState::RUNNING, arbiter.getGameState());

    gameData.ball.setLocation(Point2D(-1.5, -2.5));
    arbiter.control(gameData);
    EXPECT_EQ(GameState::RUNNING, arbiter.getGameState());

    gameData.ball.setLocation(Point2D(1.5, -2.5));
    arbiter.control(gameData);
    EXPECT_EQ(GameState::RUNNING, arbiter.getGameState());

    gameData.ball.setLocation(Point2D(1.5, 2.5));
    arbiter.control(gameData);
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
        arbiter.control(gameData);
    }
}


TEST_F(AnArbiterForARunningGame, StopsTheGameIfTheBallIsOutside)
{
    EXPECT_CALL(mockAbstractRefBoxAdapter, sendStop());

    gameData.ball.setLocation(Point2D(20.0, 20.0));

    while (arbiter.getGameState() != GameState::STOPPED)
    {
        arbiter.control(gameData);
    }
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
