// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RuleSetpieceExecuteFinishedTest.cpp
 *
 *  Created on: Sep 24, 2016
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "../TeamplayTest.hpp"

/* SUT */
#include "int/rules/RuleSetpieceExecuteFinished.hpp"

/* SUT dependencies */
#include "int/stores/BallStore.hpp"
#include "int/stores/ConfigurationStore.hpp"
#include "int/stores/GameStateStore.hpp"

class TimerMock : public teamplay::Timer
{
public:
    MOCK_METHOD0(reset, void());
    MOCK_CONST_METHOD1(hasElapsed, bool(const double));
};

class RuleSetpieceExecuteFinishedTest : public TeamplayTest
{
public:
    RuleSetpieceExecuteFinishedTest()
    {
        ConfigTeamplay config;

        config.rules.setpieceExecuteTimeout = 1;
        config.rules.penaltyExecuteTimeout = 2;
        config.rules.minKickDistanceKicked = 1;
        config.rules.minPenaltyDistanceKicked = 2;
        config.rules.minOwnKickoffDistanceKicked = 3;

        // TODO -- new configuration design does not support overrulig
        //ConfigurationStore::getConfiguration().update(config);

        timer_p.reset(new NiceMock<TimerMock>);
    }

    boost::shared_ptr<NiceMock<TimerMock>> timer_p;
};

TEST_F(RuleSetpieceExecuteFinishedTest, DroppedBallIsAlwaysImmediatelyFinished)
{
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_EXECUTING,
                                                  GoverningMatchState::IN_MATCH,
                                                  SetpieceOwner::NONE,
                                                  SetpieceType::DROPPED_BALL);
    RuleSetpieceExecuteFinished rule(timer_p);
    EXPECT_TRUE(rule.isRuleValid());
}

TEST_F(RuleSetpieceExecuteFinishedTest, OpponentKickoffIsFinishedIfTheBallHasMoved)
{
    Point3D ball_location = Point3D(1.0, 2.0, 3.0);
    BallStore::getBall().setPosition(ball_location);
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_EXECUTING,
                                                  GoverningMatchState::IN_MATCH,
                                                  SetpieceOwner::OPPONENT,
                                                  SetpieceType::KICKOFF);

    RuleSetpieceExecuteFinished rule(timer_p);

    EXPECT_FALSE(rule.isRuleValid());

    ball_location = ball_location + Vector3D(1.49, 0.0, 0.0);
    teamplay::BallStore::getBall().setPosition(ball_location);

    EXPECT_FALSE(rule.isRuleValid());

    ball_location = ball_location + Vector3D(0.02, 0.0, 0.0);
    teamplay::BallStore::getBall().setPosition(ball_location);

    EXPECT_TRUE(rule.isRuleValid());
}

TEST_F(RuleSetpieceExecuteFinishedTest, OwnFreekickIsFinishedIfTheBallHasMoved)
{
    Point3D ball_location = Point3D(1.0, 2.0, 3.0);
    BallStore::getBall().setPosition(ball_location);
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_EXECUTING,
                                                  GoverningMatchState::IN_MATCH,
                                                  SetpieceOwner::OWN,
                                                  SetpieceType::FREEKICK);

    RuleSetpieceExecuteFinished rule(timer_p);

    EXPECT_FALSE(rule.isRuleValid());

    ball_location = ball_location + Vector3D(1.49, 0.0, 0.0);
    teamplay::BallStore::getBall().setPosition(ball_location);

    EXPECT_FALSE(rule.isRuleValid());

    ball_location = ball_location + Vector3D(0.02, 0.0, 0.0);
    teamplay::BallStore::getBall().setPosition(ball_location);

    EXPECT_TRUE(rule.isRuleValid());
}

TEST_F(RuleSetpieceExecuteFinishedTest, OpponentPenaltyIsFinishedIfTheBallHasMoved)
{
    Point3D ball_location = Point3D(1.0, 2.0, 3.0);
    BallStore::getBall().setPosition(ball_location);
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_EXECUTING,
                                                  GoverningMatchState::IN_MATCH,
                                                  SetpieceOwner::OPPONENT,
                                                  SetpieceType::PENALTY);

    RuleSetpieceExecuteFinished rule(timer_p);

    EXPECT_FALSE(rule.isRuleValid());

    ball_location = ball_location + Vector3D(1.99, 0.0, 0.0);
    teamplay::BallStore::getBall().setPosition(ball_location);

    EXPECT_FALSE(rule.isRuleValid());

    ball_location = ball_location + Vector3D(0.02, 0.0, 0.0);
    teamplay::BallStore::getBall().setPosition(ball_location);

    EXPECT_TRUE(rule.isRuleValid());
}

TEST_F(RuleSetpieceExecuteFinishedTest, OwnKickoffIsFinishedIfTheBallHasMoved)
{
    Point3D ball_location = Point3D(1.0, 2.0, 3.0);
    BallStore::getBall().setPosition(ball_location);
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_EXECUTING,
                                                  GoverningMatchState::IN_MATCH,
                                                  SetpieceOwner::OWN,
                                                  SetpieceType::KICKOFF);

    RuleSetpieceExecuteFinished rule(timer_p);

    EXPECT_FALSE(rule.isRuleValid());

    ball_location = ball_location + Vector3D(1.49, 0.0, 0.0);
    teamplay::BallStore::getBall().setPosition(ball_location);

    EXPECT_FALSE(rule.isRuleValid());

    ball_location = ball_location + Vector3D(0.02, 0.0, 0.0);
    teamplay::BallStore::getBall().setPosition(ball_location);

    EXPECT_TRUE(rule.isRuleValid());
}

TEST_F(RuleSetpieceExecuteFinishedTest, OpponentKickoffIsFinishedIfTheTimerHasElapsed)
{
    Point3D ball_location = Point3D();
    BallStore::getBall().setPosition(ball_location);
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_EXECUTING,
                                                  GoverningMatchState::IN_MATCH,
                                                  SetpieceOwner::OPPONENT,
                                                  SetpieceType::KICKOFF);

    RuleSetpieceExecuteFinished rule(timer_p);

    EXPECT_CALL(*timer_p, hasElapsed(_)).WillRepeatedly(Return(false));

    EXPECT_FALSE(rule.isRuleValid());

    EXPECT_CALL(*timer_p, hasElapsed(ConfigurationStore::getConfiguration().getSetPieceExecuteTimeoutSeconds()))
               .WillRepeatedly(Return(true));

    EXPECT_TRUE(rule.isRuleValid());
}

TEST_F(RuleSetpieceExecuteFinishedTest, OwnFreekickIsFinishedIfTheTimerHasElapsed)
{
    Point3D ball_location = Point3D();
    BallStore::getBall().setPosition(ball_location);
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_EXECUTING,
                                                  GoverningMatchState::IN_MATCH,
                                                  SetpieceOwner::OWN,
                                                  SetpieceType::FREEKICK);

    RuleSetpieceExecuteFinished rule(timer_p);

    EXPECT_CALL(*timer_p, hasElapsed(_)).WillRepeatedly(Return(false));

    EXPECT_FALSE(rule.isRuleValid());

    EXPECT_CALL(*timer_p, hasElapsed(ConfigurationStore::getConfiguration().getSetPieceExecuteTimeoutSeconds()))
               .WillRepeatedly(Return(true));

    EXPECT_TRUE(rule.isRuleValid());
}

TEST_F(RuleSetpieceExecuteFinishedTest, OpponentPenaltyIsFinishedIfTheTimerHasElapsed)
{
    Point3D ball_location = Point3D();
    BallStore::getBall().setPosition(ball_location);
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_EXECUTING,
                                                  GoverningMatchState::IN_MATCH,
                                                  SetpieceOwner::OPPONENT,
                                                  SetpieceType::PENALTY);

    RuleSetpieceExecuteFinished rule(timer_p);

    EXPECT_CALL(*timer_p, hasElapsed(_)).WillRepeatedly(Return(false));

    EXPECT_FALSE(rule.isRuleValid());

    EXPECT_CALL(*timer_p, hasElapsed(ConfigurationStore::getConfiguration().getPenaltyExecuteTimeoutSeconds()))
               .WillRepeatedly(Return(true));

    EXPECT_TRUE(rule.isRuleValid());
}

TEST_F(RuleSetpieceExecuteFinishedTest, OwnKickoffIsFinishedIfTheTimerHasElapsed)
{
    Point3D ball_location = Point3D();
    BallStore::getBall().setPosition(ball_location);
    GameStateStore::getInstance().updateGameState(GoverningGameState::SETPIECE_EXECUTING,
                                                  GoverningMatchState::IN_MATCH,
                                                  SetpieceOwner::OWN,
                                                  SetpieceType::KICKOFF);

    RuleSetpieceExecuteFinished rule(timer_p);

    EXPECT_CALL(*timer_p, hasElapsed(_)).WillRepeatedly(Return(false));

    EXPECT_FALSE(rule.isRuleValid());

    EXPECT_CALL(*timer_p, hasElapsed(ConfigurationStore::getConfiguration().getSetPieceExecuteTimeoutSeconds()))
               .WillRepeatedly(Return(true));

    EXPECT_TRUE(rule.isRuleValid());
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
