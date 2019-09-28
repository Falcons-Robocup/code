 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ruleAvoidAreasTest.cpp
 *
 *  Created on: Apr 24, 2017
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "teamplayTest.hpp"

/* SUT */
#include "int/rules/ruleAvoidAreas.hpp"

/* SUT dependencies */
#include "int/stores/robotStore.hpp"

class TimerMock : public teamplay::timer
{
public:
    MOCK_METHOD0(reset, void());
    MOCK_CONST_METHOD1(hasElapsed, bool(const double));
};

static void setOwnRobotPosition (const Position2D pos)
{
    robotStore::getInstance().addOwnRobot(robot(0, treeEnum::R_ROBOT_STOP, pos, Velocity2D()));
}

class ownPenaltyArea : public TeamplayTest
{
public:
    ownPenaltyArea()
    : timer_p(new NiceMock<TimerMock>)
    , rule(ruleAvoidAreas(timer_p))
    {
        robotStore::getInstance().clear();
        setOwnRobotPosition(Position2D(0.0, -8.0, 0.0));
    }

    boost::shared_ptr<NiceMock<TimerMock>> timer_p;
    ruleAvoidAreas rule;
};

class NonGoalieInOwnPenaltyArea : public ownPenaltyArea
{
public:
    NonGoalieInOwnPenaltyArea()
    {
        robotStore::getInstance().setOwnRobotRole(treeEnum::DEFENDER_MAIN);
        robotStore::getInstance().addTeammate(
                        robot(1, treeEnum::R_GOALKEEPER, Position2D(0.0, -8.8, 0.0), Velocity2D(0.0, 0.0, 0.0)));
    }
};

class GoalieInOwnPenaltyArea : public ownPenaltyArea
{
public:
    GoalieInOwnPenaltyArea()
    {
        robotStore::getInstance().setOwnRobotRole(treeEnum::R_GOALKEEPER);
        robotStore::getInstance().addTeammate(
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
        robotStore::getInstance().clear();
        setOwnRobotPosition(Position2D(0, 8, 0));
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
        robotStore::getInstance().clear();
        setOwnRobotPosition(Position2D(0, -8.9, 0));
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
        robotStore::getInstance().clear();
        setOwnRobotPosition(Position2D(0, 8.9, 0));
    }

    boost::shared_ptr<NiceMock<TimerMock>> timer_p;
    ruleAvoidAreas rule;
};

class nearOwnPenaltyArea : public TeamplayTest
{
public:
    nearOwnPenaltyArea()
    : timer_p(new NiceMock<TimerMock>)
    , rule(ruleAvoidAreas(timer_p))
    {
        robotStore::getInstance().clear();
        setOwnRobotPosition(Position2D(0, -7, 0));
    }

    boost::shared_ptr<NiceMock<TimerMock>> timer_p;
    ruleAvoidAreas rule;
};

class nearOppPenaltyArea : public TeamplayTest
{
public:
    nearOppPenaltyArea()
    : timer_p(new NiceMock<TimerMock>)
    , rule(ruleAvoidAreas(timer_p))
    {
        robotStore::getInstance().clear();
        setOwnRobotPosition(Position2D(0, 7, 0));
    }

    boost::shared_ptr<NiceMock<TimerMock>> timer_p;
    ruleAvoidAreas rule;
};

class inCenterArea : public TeamplayTest
{
public:
    inCenterArea()
    : timer_p(new NiceMock<TimerMock>)
    , rule(ruleAvoidAreas(timer_p))
    {
        robotStore::getInstance().clear();
        setOwnRobotPosition(Position2D(0, 0, 0));
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
    robotStore::getInstance().addTeammate(robot(2, treeEnum::DEFENDER_ASSIST, Position2D(0.0, -5.0, 0.0), Velocity2D(0.0, 0.0, 0.0)));

    EXPECT_TRUE(rule.isCurrentPositionValid());
}

TEST_F(NonGoalieInOwnPenaltyArea, CurrentPositionIsInvalidIfAreaIsOccupied)
{
    robotStore::getInstance().addTeammate(robot(2, treeEnum::DEFENDER_ASSIST, Position2D(0.0, -8.0, 0.0), Velocity2D(0.0, 0.0, 0.0)));

    EXPECT_FALSE(rule.isCurrentPositionValid());
}

TEST_F(nearOwnPenaltyArea, TargetPositionIsValidIfAreaOnlyOccupiedByGoalie)
{
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
    // since May 2019, Teamplay now always sets forbidden areas on (extended) goal area, which was previously done by pathPlanning
}

TEST_F(nearOwnPenaltyArea, TargetPositionIsInvalidIfAreaIsOccupied)
{
    robotStore::getInstance().addTeammate(robot(2, treeEnum::DEFENDER_ASSIST, Position2D(0.0, -8.0, 0.0), Velocity2D(0.0, 0.0, 0.0)));
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
}

TEST_F(inCenterArea, TargetPositionIsValidIfCurrentPositionIsFarAway)
{
    robotStore::getInstance().addTeammate(robot(2, treeEnum::DEFENDER_ASSIST, Position2D(0.0, -8.0, 0.0), Velocity2D(0.0, 0.0, 0.0)));
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
    // since May 2019, Teamplay now always sets forbidden areas on (extended) goal area, which was previously done by pathPlanning
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
    robotStore::getInstance().addTeammate(robot(2, treeEnum::DEFENDER_ASSIST, Position2D(0.0, -5.0, 0.0), Velocity2D(0.0, 0.0, 0.0)));

    EXPECT_TRUE(rule.isCurrentPositionValid());
}

TEST_F(GoalieInOwnPenaltyArea, CurrentPositionIsValidIfAreaIsOccupied)
{
    robotStore::getInstance().addTeammate(robot(2, treeEnum::DEFENDER_ASSIST, Position2D(0.0, -8.0, 0.0), Velocity2D(0.0, 0.0, 0.0)));

    EXPECT_TRUE(rule.isCurrentPositionValid());
}


TEST_F(inOppPenaltyArea, CurrentPositionIsInvalidAfterTimerElapsed)
{
    EXPECT_CALL(*timer_p, hasElapsed(_)).WillRepeatedly(Return(true));

    EXPECT_FALSE(rule.isCurrentPositionValid());
}

TEST_F(inOppPenaltyArea, CurrentPositionIsValidIfAreaIsEmpty)
{
    robotStore::getInstance().addTeammate(robot(2, treeEnum::DEFENDER_ASSIST, Position2D(0.0, 5.0, 0.0), Velocity2D(0.0, 0.0, 0.0)));

    EXPECT_TRUE(rule.isCurrentPositionValid());
}

TEST_F(inOppPenaltyArea, CurrentPositionIsInvalidIfAreaIsOccupied)
{
    robotStore::getInstance().addTeammate(robot(2, treeEnum::DEFENDER_ASSIST, Position2D(0.0, 8.0, 0.0), Velocity2D(0.0, 0.0, 0.0)));

    EXPECT_FALSE(rule.isCurrentPositionValid());
}

TEST_F(nearOppPenaltyArea, TargetPositionIsValidIfAreaEmpty)
{
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
    // since May 2019, Teamplay now always sets forbidden areas on (extended) goal area, which was previously done by pathPlanning
}

TEST_F(nearOppPenaltyArea, TargetPositionIsInvalidIfAreaIsOccupied)
{
    robotStore::getInstance().addTeammate(robot(2, treeEnum::DEFENDER_ASSIST, Position2D(0.0, 8.0, 0.0), Velocity2D(0.0, 0.0, 0.0)));
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
}

TEST_F(inOwnGoalArea, CurrentPositionIsValidForGoalie)
{
    robotStore::getInstance().setOwnRobotRole(treeEnum::R_GOALKEEPER);
    EXPECT_TRUE(rule.isCurrentPositionValid());
}

TEST_F(inOwnGoalArea, CurrentPositionIsInvalidForNonGoalie)
{
    robotStore::getInstance().setOwnRobotRole(treeEnum::DEFENDER_MAIN);
    EXPECT_FALSE(rule.isCurrentPositionValid());
}

TEST_F(inOppGoalArea, CurrentPositionIsInvalidForGoalie)
{
    robotStore::getInstance().setOwnRobotRole(treeEnum::R_GOALKEEPER);
    EXPECT_FALSE(rule.isCurrentPositionValid());
}

TEST_F(inOppGoalArea, CurrentPositionIsInvalidForNonGoalie)
{
    robotStore::getInstance().setOwnRobotRole(treeEnum::DEFENDER_MAIN);
    EXPECT_FALSE(rule.isCurrentPositionValid());
}

TEST_F(inOwnGoalArea, TargetPositionIsValidForGoalie)
{
    robotStore::getInstance().setOwnRobotRole(treeEnum::R_GOALKEEPER);
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
    // since May 2019, Teamplay now always sets forbidden areas on (extended) goal area, which was previously done by pathPlanning
    // in this case, it will not get a forbidden area on own goal area, but always on opponent area, although this is hardly relevant for goalie
}

TEST_F(inOwnGoalArea, TargetPositionIsInvalidForNonGoalie)
{
    robotStore::getInstance().setOwnRobotRole(treeEnum::DEFENDER_MAIN);
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
}

TEST_F(inOppGoalArea, TargetPositionIsInvalidForGoalie)
{
    robotStore::getInstance().setOwnRobotRole(treeEnum::R_GOALKEEPER);
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
}

TEST_F(inOppGoalArea, TargetPositionIsInvalidForNonGoalie)
{
    robotStore::getInstance().setOwnRobotRole(treeEnum::DEFENDER_MAIN);
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
