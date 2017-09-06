 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
        configurationStore::getConfiguration().setSetPieceExecuteTimeout(1);
        configurationStore::getConfiguration().setPenaltyExecuteTimeout(2);
        configurationStore::getConfiguration().setMinKickDistanceKickedMeters(1);
        configurationStore::getConfiguration().setMinPenaltyDistanceKickedMeters(2);
        configurationStore::getConfiguration().setMinOwnKickoffDistanceKickedMeters(3);

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
