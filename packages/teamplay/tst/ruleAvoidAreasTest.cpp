 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ownPenaltyArea.cpp
 *
 *  Created on: Apr 24, 2017
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "teamplayTest.hpp"

/* SUT */
#include "int/rules/ruleAvoidAreas.hpp"

/* SUT dependencies */
#include "int/stores/ownRobotStore.hpp"
#include "int/stores/teamMatesStore.hpp"

class TimerMock : public teamplay::timer
{
public:
    MOCK_METHOD0(reset, void());
    MOCK_CONST_METHOD1(hasElapsed, bool(const double));
};

class ownPenaltyArea : public TeamplayTest
{
public:
    ownPenaltyArea()
    : timer_p(new NiceMock<TimerMock>)
    , rule(ruleAvoidAreas(timer_p))
    {
        ownRobotStore::getOwnRobot().setPosition(Position2D(0.0, -8.0, 0.0));
        teamMatesStore::getTeamMatesIncludingGoalie().clear();
    }

    boost::shared_ptr<NiceMock<TimerMock>> timer_p;
    ruleAvoidAreas rule;
};

class NonGoalieInOwnPenaltyArea : public ownPenaltyArea
{
public:
    NonGoalieInOwnPenaltyArea()
    {
        ownRobotStore::getOwnRobot().setRole(treeEnum::DEFENDER_MAIN);
        teamMatesStore::getTeamMatesIncludingGoalie().add(
                        robot(1, treeEnum::R_GOALKEEPER, Position2D(0.0, -8.8, 0.0), Velocity2D(0.0, 0.0, 0.0)));
    }
};

class GoalieInOwnPenaltyArea : public ownPenaltyArea
{
public:
    GoalieInOwnPenaltyArea()
    {
        ownRobotStore::getOwnRobot().setRole(treeEnum::R_GOALKEEPER);
        teamMatesStore::getTeamMatesIncludingGoalie().add(
                        robot(2, treeEnum::DEFENDER_MAIN, Position2D(0.0, 0.0, 0.0), Velocity2D(0.0, 0.0, 0.0)));
    }
};

class inOppPenaltyArea : public TeamplayTest
{
public:
    inOppPenaltyArea()
    : timer_p(new NiceMock<TimerMock>)
    , rule(ruleAvoidAreas(timer_p))
    {
        ownRobotStore::getOwnRobot().setPosition(Position2D(0.0, 8.0, 0.0));
        teamMatesStore::getTeamMatesIncludingGoalie().clear();
    }

    boost::shared_ptr<NiceMock<TimerMock>> timer_p;
    ruleAvoidAreas rule;
};

class inOwnGoalArea : public TeamplayTest
{
public:
    inOwnGoalArea()
    : timer_p(new NiceMock<TimerMock>)
    , rule(ruleAvoidAreas(timer_p))
    {
        ownRobotStore::getOwnRobot().setPosition(Position2D(0.0, -8.9, 0.0));
        teamMatesStore::getTeamMatesIncludingGoalie().clear();
    }

    boost::shared_ptr<NiceMock<TimerMock>> timer_p;
    ruleAvoidAreas rule;
};

class inOppGoalArea : public TeamplayTest
{
public:
    inOppGoalArea()
    : timer_p(new NiceMock<TimerMock>)
    , rule(ruleAvoidAreas(timer_p))
    {
        ownRobotStore::getOwnRobot().setPosition(Position2D(0.0, 8.9, 0.0));
        teamMatesStore::getTeamMatesIncludingGoalie().clear();
    }

    boost::shared_ptr<NiceMock<TimerMock>> timer_p;
    ruleAvoidAreas rule;
};


TEST_F(NonGoalieInOwnPenaltyArea, CurrentPositionIsValidIfAreaOnlyOccupiedByGoalie)
{
    EXPECT_TRUE(rule.isCurrentPositionValid());
}

TEST_F(NonGoalieInOwnPenaltyArea, CurrentPositionIsInvalidAfterTimerElapsed)
{
    EXPECT_CALL(*timer_p, hasElapsed(_)).WillRepeatedly(Return(true));

    EXPECT_FALSE(rule.isCurrentPositionValid());
}

TEST_F(NonGoalieInOwnPenaltyArea, CurrentPositionIsValidIfAreaIsEmpty)
{
    teamMatesStore::getTeamMatesIncludingGoalie().add(robot(2, treeEnum::DEFENDER_ASSIST, Position2D(0.0, -5.0, 0.0), Velocity2D(0.0, 0.0, 0.0)));

    EXPECT_TRUE(rule.isCurrentPositionValid());
}

TEST_F(NonGoalieInOwnPenaltyArea, CurrentPositionIsInvalidIfAreaIsOccupied)
{
    teamMatesStore::getTeamMatesIncludingGoalie().add(robot(2, treeEnum::DEFENDER_ASSIST, Position2D(0.0, -8.0, 0.0), Velocity2D(0.0, 0.0, 0.0)));

    EXPECT_FALSE(rule.isCurrentPositionValid());
}

TEST_F(NonGoalieInOwnPenaltyArea, TargetPositionIsValidIfAreaOnlyOccupiedByGoalie)
{
    ownRobotStore::getOwnRobot().setPosition(Position2D(0.0, -7.0, 0.0));
    EXPECT_TRUE(rule.getForbiddenAreas().empty());
}

TEST_F(NonGoalieInOwnPenaltyArea, TargetPositionIsInvalidIfAreaIsOccupied)
{
    teamMatesStore::getTeamMatesIncludingGoalie().add(robot(2, treeEnum::DEFENDER_ASSIST, Position2D(0.0, -8.0, 0.0), Velocity2D(0.0, 0.0, 0.0)));

    ownRobotStore::getOwnRobot().setPosition(Position2D(0.0, -7.0, 0.0));
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
}

TEST_F(NonGoalieInOwnPenaltyArea, TargetPositionIsValidIfCurrentPositionIsFarAway)
{
    teamMatesStore::getTeamMatesIncludingGoalie().add(robot(2, treeEnum::DEFENDER_ASSIST, Position2D(0.0, -8.0, 0.0), Velocity2D(0.0, 0.0, 0.0)));
    ownRobotStore::getOwnRobot().setPosition(Position2D(0.0, -5.0, 0.0));
    EXPECT_TRUE(rule.getForbiddenAreas().empty());
}


TEST_F(GoalieInOwnPenaltyArea, CurrentPositionIsValidIfAreaOnlyOccupiedByGoalie)
{
    EXPECT_TRUE(rule.isCurrentPositionValid());
}

TEST_F(GoalieInOwnPenaltyArea, CurrentPositionIsValidAfterTimerElapsed)
{
    EXPECT_CALL(*timer_p, hasElapsed(_)).WillRepeatedly(Return(true));

    EXPECT_TRUE(rule.isCurrentPositionValid());
}

TEST_F(GoalieInOwnPenaltyArea, CurrentPositionIsValidIfAreaIsEmpty)
{
    teamMatesStore::getTeamMatesIncludingGoalie().add(robot(2, treeEnum::DEFENDER_ASSIST, Position2D(0.0, -5.0, 0.0), Velocity2D(0.0, 0.0, 0.0)));

    EXPECT_TRUE(rule.isCurrentPositionValid());
}

TEST_F(GoalieInOwnPenaltyArea, CurrentPositionIsValidIfAreaIsOccupied)
{
    teamMatesStore::getTeamMatesIncludingGoalie().add(robot(2, treeEnum::DEFENDER_ASSIST, Position2D(0.0, -8.0, 0.0), Velocity2D(0.0, 0.0, 0.0)));

    EXPECT_TRUE(rule.isCurrentPositionValid());
}


TEST_F(inOppPenaltyArea, CurrentPositionIsInvalidAfterTimerElapsed)
{
    EXPECT_CALL(*timer_p, hasElapsed(_)).WillRepeatedly(Return(true));

    EXPECT_FALSE(rule.isCurrentPositionValid());
}

TEST_F(inOppPenaltyArea, CurrentPositionIsValidIfAreaIsEmpty)
{
    teamMatesStore::getTeamMatesIncludingGoalie().add(robot(2, treeEnum::DEFENDER_ASSIST, Position2D(0.0, 5.0, 0.0), Velocity2D(0.0, 0.0, 0.0)));

    EXPECT_TRUE(rule.isCurrentPositionValid());
}

TEST_F(inOppPenaltyArea, CurrentPositionIsInvalidIfAreaIsOccupied)
{
    teamMatesStore::getTeamMatesIncludingGoalie().add(robot(2, treeEnum::DEFENDER_ASSIST, Position2D(0.0, 8.0, 0.0), Velocity2D(0.0, 0.0, 0.0)));

    EXPECT_FALSE(rule.isCurrentPositionValid());
}

TEST_F(inOppPenaltyArea, TargetPositionIsValidIfAreaEmpty)
{
    ownRobotStore::getOwnRobot().setPosition(Position2D(0.0, 7.0, 0.0));
    EXPECT_TRUE(rule.getForbiddenAreas().empty());
}

TEST_F(inOppPenaltyArea, TargetPositionIsInvalidIfAreaIsOccupied)
{
    teamMatesStore::getTeamMatesIncludingGoalie().add(robot(2, treeEnum::DEFENDER_ASSIST, Position2D(0.0, 8.0, 0.0), Velocity2D(0.0, 0.0, 0.0)));

    ownRobotStore::getOwnRobot().setPosition(Position2D(0.0, 7.0, 0.0));
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
}

TEST_F(inOppPenaltyArea, TargetPositionIsValidIfCurrentPositionIsFarAway)
{
    teamMatesStore::getTeamMatesIncludingGoalie().add(robot(2, treeEnum::DEFENDER_ASSIST, Position2D(0.0, 8.0, 0.0), Velocity2D(0.0, 0.0, 0.0)));
    ownRobotStore::getOwnRobot().setPosition(Position2D(0.0, 5.0, 0.0));
    EXPECT_TRUE(rule.getForbiddenAreas().empty());
}

TEST_F(inOwnGoalArea, CurrentPositionIsValidForGoalie)
{
    ownRobotStore::getOwnRobot().setRole(treeEnum::R_GOALKEEPER);
    EXPECT_TRUE(rule.isCurrentPositionValid());
}

TEST_F(inOwnGoalArea, CurrentPositionIsInvalidForNonGoalie)
{
    ownRobotStore::getOwnRobot().setRole(treeEnum::DEFENDER_MAIN);
    EXPECT_FALSE(rule.isCurrentPositionValid());
}

TEST_F(inOppGoalArea, CurrentPositionIsInvalidForGoalie)
{
    ownRobotStore::getOwnRobot().setRole(treeEnum::R_GOALKEEPER);
    EXPECT_FALSE(rule.isCurrentPositionValid());
}

TEST_F(inOppGoalArea, CurrentPositionIsInvalidForNonGoalie)
{
    ownRobotStore::getOwnRobot().setRole(treeEnum::DEFENDER_MAIN);
    EXPECT_FALSE(rule.isCurrentPositionValid());
}

TEST_F(inOwnGoalArea, TargetPositionIsValidForGoalie)
{
    ownRobotStore::getOwnRobot().setRole(treeEnum::R_GOALKEEPER);
    ownRobotStore::getOwnRobot().setPosition(Position2D(0.0, -8.8, 0.0));
    EXPECT_TRUE(rule.getForbiddenAreas().empty());
}

TEST_F(inOwnGoalArea, TargetPositionIsInvalidForNonGoalie)
{
    ownRobotStore::getOwnRobot().setRole(treeEnum::DEFENDER_MAIN);
    ownRobotStore::getOwnRobot().setPosition(Position2D(0.0, -8.8, 0.0));
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
}

TEST_F(inOwnGoalArea, TargetPositionIsValidIfCurrentPositionIsFarAway)
{
    ownRobotStore::getOwnRobot().setRole(treeEnum::DEFENDER_MAIN);
    ownRobotStore::getOwnRobot().setPosition(Position2D(0.0, -6.0, 0.0));
    EXPECT_TRUE(rule.getForbiddenAreas().empty());
}

TEST_F(inOppGoalArea, TargetPositionIsInvalidForGoalie)
{
    ownRobotStore::getOwnRobot().setRole(treeEnum::R_GOALKEEPER);
    ownRobotStore::getOwnRobot().setPosition(Position2D(0.0, 8.8, 0.0));
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
}

TEST_F(inOppGoalArea, TargetPositionIsInvalidForNonGoalie)
{
    ownRobotStore::getOwnRobot().setRole(treeEnum::DEFENDER_MAIN);
    ownRobotStore::getOwnRobot().setPosition(Position2D(0.0, 8.8, 0.0));
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
}

TEST_F(inOppGoalArea, TargetPositionIsValidIfCurrentPositionIsFarAway)
{
    ownRobotStore::getOwnRobot().setRole(treeEnum::DEFENDER_MAIN);
    ownRobotStore::getOwnRobot().setPosition(Position2D(0.0, 6.0, 0.0));
    EXPECT_TRUE(rule.getForbiddenAreas().empty());
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
