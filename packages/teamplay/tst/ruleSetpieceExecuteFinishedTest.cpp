// Copyright 2017-2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ruleSetpieceExecuteFinishedTest.cpp
 *
 *  Created on: Sep 24, 2016
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "teamplayTest.hpp"

/* SUT */
#include "int/rules/ruleSetpieceExecuteFinished.hpp"

/* SUT dependencies */
#include "int/stores/ballStore.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/stores/gameStateStore.hpp"

class TimerMock : public teamplay::timer
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
        configTeamplay config;

        config.rules.setpieceExecuteTimeout = 1;
        config.rules.penaltyExecuteTimeout = 2;
        config.rules.minKickDistanceKicked = 1;
        config.rules.minPenaltyDistanceKicked = 2;
        config.rules.minOwnKickoffDistanceKicked = 3;

        configurationStore::getConfiguration().update(config);

        timer_p.reset(new NiceMock<TimerMock>);
    }

    boost::shared_ptr<NiceMock<TimerMock>> timer_p;
};

TEST_F(RuleSetpieceExecuteFinishedTest, DroppedBallIsAlwaysImmediatelyFinished)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::NONE, setpieceType::DROPPED_BALL);
    ruleSetpieceExecuteFinished rule(timer_p);
    EXPECT_TRUE(rule.isRuleValid());
}

TEST_F(RuleSetpieceExecuteFinishedTest, OpponentKickoffIsFinishedIfTheBallHasMoved)
{
    Point3D ball_location = Point3D(1.0, 2.0, 3.0);
    ballStore::getBall().setPosition(ball_location);
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OPPONENT, setpieceType::KICKOFF);

    ruleSetpieceExecuteFinished rule(timer_p);

    EXPECT_FALSE(rule.isRuleValid());

    ball_location = ball_location + Vector3D(0.99, 0.0, 0.0);
    teamplay::ballStore::getBall().setPosition(ball_location);

    EXPECT_FALSE(rule.isRuleValid());

    ball_location = ball_location + Vector3D(0.02, 0.0, 0.0);
    teamplay::ballStore::getBall().setPosition(ball_location);

    EXPECT_TRUE(rule.isRuleValid());
}

TEST_F(RuleSetpieceExecuteFinishedTest, OwnFreekickIsFinishedIfTheBallHasMoved)
{
    Point3D ball_location = Point3D(1.0, 2.0, 3.0);
    ballStore::getBall().setPosition(ball_location);
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OWN, setpieceType::FREEKICK);

    ruleSetpieceExecuteFinished rule(timer_p);

    EXPECT_FALSE(rule.isRuleValid());

    ball_location = ball_location + Vector3D(0.99, 0.0, 0.0);
    teamplay::ballStore::getBall().setPosition(ball_location);

    EXPECT_FALSE(rule.isRuleValid());

    ball_location = ball_location + Vector3D(0.02, 0.0, 0.0);
    teamplay::ballStore::getBall().setPosition(ball_location);

    EXPECT_TRUE(rule.isRuleValid());
}

TEST_F(RuleSetpieceExecuteFinishedTest, OpponentPenaltyIsFinishedIfTheBallHasMoved)
{
    Point3D ball_location = Point3D(1.0, 2.0, 3.0);
    ballStore::getBall().setPosition(ball_location);
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OPPONENT, setpieceType::PENALTY);

    ruleSetpieceExecuteFinished rule(timer_p);

    EXPECT_FALSE(rule.isRuleValid());

    ball_location = ball_location + Vector3D(1.99, 0.0, 0.0);
    teamplay::ballStore::getBall().setPosition(ball_location);

    EXPECT_FALSE(rule.isRuleValid());

    ball_location = ball_location + Vector3D(0.02, 0.0, 0.0);
    teamplay::ballStore::getBall().setPosition(ball_location);

    EXPECT_TRUE(rule.isRuleValid());
}

TEST_F(RuleSetpieceExecuteFinishedTest, OwnKickoffIsFinishedIfTheBallHasMoved)
{
    Point3D ball_location = Point3D(1.0, 2.0, 3.0);
    ballStore::getBall().setPosition(ball_location);
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OWN, setpieceType::KICKOFF);

    ruleSetpieceExecuteFinished rule(timer_p);

    EXPECT_FALSE(rule.isRuleValid());

    ball_location = ball_location + Vector3D(2.99, 0.0, 0.0);
    teamplay::ballStore::getBall().setPosition(ball_location);

    EXPECT_FALSE(rule.isRuleValid());

    ball_location = ball_location + Vector3D(0.02, 0.0, 0.0);
    teamplay::ballStore::getBall().setPosition(ball_location);

    EXPECT_TRUE(rule.isRuleValid());
}

TEST_F(RuleSetpieceExecuteFinishedTest, OpponentKickoffIsFinishedIfTheTimerHasElapsed)
{
    Point3D ball_location = Point3D();
    ballStore::getBall().setPosition(ball_location);
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OPPONENT, setpieceType::KICKOFF);

    ruleSetpieceExecuteFinished rule(timer_p);

    EXPECT_CALL(*timer_p, hasElapsed(_)).WillRepeatedly(Return(false));

    EXPECT_FALSE(rule.isRuleValid());

    EXPECT_CALL(*timer_p, hasElapsed(configurationStore::getConfiguration().getSetPieceExecuteTimeoutSeconds()))
               .WillRepeatedly(Return(true));

    EXPECT_TRUE(rule.isRuleValid());
}

TEST_F(RuleSetpieceExecuteFinishedTest, OwnFreekickIsFinishedIfTheTimerHasElapsed)
{
    Point3D ball_location = Point3D();
    ballStore::getBall().setPosition(ball_location);
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OWN, setpieceType::FREEKICK);

    ruleSetpieceExecuteFinished rule(timer_p);

    EXPECT_CALL(*timer_p, hasElapsed(_)).WillRepeatedly(Return(false));

    EXPECT_FALSE(rule.isRuleValid());

    EXPECT_CALL(*timer_p, hasElapsed(configurationStore::getConfiguration().getSetPieceExecuteTimeoutSeconds()))
               .WillRepeatedly(Return(true));

    EXPECT_TRUE(rule.isRuleValid());
}

TEST_F(RuleSetpieceExecuteFinishedTest, OpponentPenaltyIsFinishedIfTheTimerHasElapsed)
{
    Point3D ball_location = Point3D();
    ballStore::getBall().setPosition(ball_location);
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OPPONENT, setpieceType::PENALTY);

    ruleSetpieceExecuteFinished rule(timer_p);

    EXPECT_CALL(*timer_p, hasElapsed(_)).WillRepeatedly(Return(false));

    EXPECT_FALSE(rule.isRuleValid());

    EXPECT_CALL(*timer_p, hasElapsed(configurationStore::getConfiguration().getPenaltyExecuteTimeoutSeconds()))
               .WillRepeatedly(Return(true));

    EXPECT_TRUE(rule.isRuleValid());
}

TEST_F(RuleSetpieceExecuteFinishedTest, OwnKickoffIsFinishedIfTheTimerHasElapsed)
{
    Point3D ball_location = Point3D();
    ballStore::getBall().setPosition(ball_location);
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OWN, setpieceType::KICKOFF);

    ruleSetpieceExecuteFinished rule(timer_p);

    EXPECT_CALL(*timer_p, hasElapsed(_)).WillRepeatedly(Return(false));

    EXPECT_FALSE(rule.isRuleValid());

    EXPECT_CALL(*timer_p, hasElapsed(configurationStore::getConfiguration().getSetPieceExecuteTimeoutSeconds()))
               .WillRepeatedly(Return(true));

    EXPECT_TRUE(rule.isRuleValid());
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
