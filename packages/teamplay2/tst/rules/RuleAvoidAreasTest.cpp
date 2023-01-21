// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RuleAvoidAreasTest.cpp
 *
 *  Created on: Apr 24, 2017
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "../TeamplayTest.hpp"

/* SUT */
#include "int/rules/RuleAvoidAreas.hpp"

/* SUT dependencies */
#include "int/stores/RobotStore.hpp"
#include "cEnvironmentField.hpp"

class TimerMock : public teamplay::Timer
{
public:
    MOCK_METHOD0(reset, void());
    MOCK_CONST_METHOD1(hasElapsed, bool(const double));
};

static void setOwnRobotPosition (const geometry::Pose2D pos)
{
    RobotStore::getInstance().addOwnRobot(Robot(0, RoleEnum::STOP, pos, geometry::Velocity2D()));
}

class ownPenaltyArea : public TeamplayTest
{
public:
    ownPenaltyArea()
    : timer_p(new NiceMock<TimerMock>)
    , rule(RuleAvoidAreas(timer_p))
    {
        RobotStore::getInstance().clear();
        setOwnRobotPosition(geometry::Pose2D(0.0, -8.0, 0.0));
    }

    boost::shared_ptr<NiceMock<TimerMock>> timer_p;
    RuleAvoidAreas rule;
};

class NonGoalieInOwnPenaltyArea : public ownPenaltyArea
{
public:
    NonGoalieInOwnPenaltyArea()
    {
        RobotStore::getInstance().setOwnRobotRole(RoleEnum::DEFENDER_MAIN);
        RobotStore::getInstance().addTeammate(
                        Robot(1, RoleEnum::GOALKEEPER, geometry::Pose2D(0.0, -8.8, 0.0), geometry::Velocity2D(0.0, 0.0, 0.0)));
    }
};

class GoalieInOwnPenaltyArea : public ownPenaltyArea
{
public:
    GoalieInOwnPenaltyArea()
    {
        RobotStore::getInstance().setOwnRobotRole(RoleEnum::GOALKEEPER);
        RobotStore::getInstance().addTeammate(
                        Robot(2, RoleEnum::DEFENDER_MAIN, geometry::Pose2D(0.0, 0.0, 0.0), geometry::Velocity2D(0.0, 0.0, 0.0)));
    }
};

class inOppPenaltyArea : public TeamplayTest
{
public:
    inOppPenaltyArea()
    : timer_p(new NiceMock<TimerMock>)
    , rule(RuleAvoidAreas(timer_p))
    {
        RobotStore::getInstance().clear();
        setOwnRobotPosition(geometry::Pose2D(0, 8, 0));
    }

    boost::shared_ptr<NiceMock<TimerMock>> timer_p;
    RuleAvoidAreas rule;
};

class inOwnGoalArea : public TeamplayTest
{
public:
    inOwnGoalArea()
    : timer_p(new NiceMock<TimerMock>)
    , rule(RuleAvoidAreas(timer_p))
    {
        RobotStore::getInstance().clear();
        setOwnRobotPosition(geometry::Pose2D(0, -8.9, 0));
    }

    boost::shared_ptr<NiceMock<TimerMock>> timer_p;
    RuleAvoidAreas rule;
};

class inOppGoalArea : public TeamplayTest
{
public:
    inOppGoalArea()
    : timer_p(new NiceMock<TimerMock>)
    , rule(RuleAvoidAreas(timer_p))
    {
        RobotStore::getInstance().clear();
        setOwnRobotPosition(geometry::Pose2D(0, 8.9, 0));
    }

    boost::shared_ptr<NiceMock<TimerMock>> timer_p;
    RuleAvoidAreas rule;
};

class nearOwnPenaltyArea : public TeamplayTest
{
public:
    nearOwnPenaltyArea()
    : timer_p(new NiceMock<TimerMock>)
    , rule(RuleAvoidAreas(timer_p))
    {
        RobotStore::getInstance().clear();
        setOwnRobotPosition(geometry::Pose2D(0, -7, 0));
    }

    boost::shared_ptr<NiceMock<TimerMock>> timer_p;
    RuleAvoidAreas rule;
};

class nearOppPenaltyArea : public TeamplayTest
{
public:
    nearOppPenaltyArea()
    : timer_p(new NiceMock<TimerMock>)
    , rule(RuleAvoidAreas(timer_p))
    {
        RobotStore::getInstance().clear();
        setOwnRobotPosition(geometry::Pose2D(0, 7, 0));
    }

    boost::shared_ptr<NiceMock<TimerMock>> timer_p;
    RuleAvoidAreas rule;
};

class inCenterArea : public TeamplayTest
{
public:
    inCenterArea()
    : timer_p(new NiceMock<TimerMock>)
    , rule(RuleAvoidAreas(timer_p))
    {
        RobotStore::getInstance().clear();
        setOwnRobotPosition(geometry::Pose2D(0, 0, 0));
    }

    boost::shared_ptr<NiceMock<TimerMock>> timer_p;
    RuleAvoidAreas rule;
};


TEST_F(NonGoalieInOwnPenaltyArea, CurrentPositionIsValidIfAreaOnlyOccupiedByGoalie)
{
    EXPECT_TRUE(rule.isCurrentPositionValid());
}

TEST_F(NonGoalieInOwnPenaltyArea, CurrentPositionIsInvalidAfterTimerElapsed)
{
    cEnvironmentField::getInstance().loadConfig("cEnvironment12x18");

    EXPECT_CALL(*timer_p, hasElapsed(_)).WillRepeatedly(Return(true));

    EXPECT_FALSE(rule.isCurrentPositionValid());
}

TEST_F(NonGoalieInOwnPenaltyArea, CurrentPositionIsValidIfAreaIsEmpty)
{
    RobotStore::getInstance().addTeammate(Robot(2, RoleEnum::DEFENDER_ASSIST, geometry::Pose2D(0.0, -5.0, 0.0), geometry::Velocity2D(0.0, 0.0, 0.0)));

    EXPECT_TRUE(rule.isCurrentPositionValid());
}

TEST_F(NonGoalieInOwnPenaltyArea, CurrentPositionIsInvalidIfAreaIsOccupied)
{
    cEnvironmentField::getInstance().loadConfig("cEnvironment12x18");

    RobotStore::getInstance().addTeammate(Robot(2, RoleEnum::DEFENDER_ASSIST, geometry::Pose2D(0.0, -8.0, 0.0), geometry::Velocity2D(0.0, 0.0, 0.0)));

    EXPECT_FALSE(rule.isCurrentPositionValid());
}

TEST_F(nearOwnPenaltyArea, TargetPositionIsValidIfAreaOnlyOccupiedByGoalie)
{
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
    // since May 2019, Teamplay now always sets forbidden areas on (extended) goal area, which was previously done by pathPlanning
}

TEST_F(nearOwnPenaltyArea, TargetPositionIsInvalidIfAreaIsOccupied)
{
    RobotStore::getInstance().addTeammate(Robot(2, RoleEnum::DEFENDER_ASSIST, geometry::Pose2D(0.0, -8.0, 0.0), geometry::Velocity2D(0.0, 0.0, 0.0)));
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
}

TEST_F(inCenterArea, TargetPositionIsValidIfCurrentPositionIsFarAway)
{
    RobotStore::getInstance().addTeammate(Robot(2, RoleEnum::DEFENDER_ASSIST, geometry::Pose2D(0.0, -8.0, 0.0), geometry::Velocity2D(0.0, 0.0, 0.0)));
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
    RobotStore::getInstance().addTeammate(Robot(2, RoleEnum::DEFENDER_ASSIST, geometry::Pose2D(0.0, -5.0, 0.0), geometry::Velocity2D(0.0, 0.0, 0.0)));

    EXPECT_TRUE(rule.isCurrentPositionValid());
}

TEST_F(GoalieInOwnPenaltyArea, CurrentPositionIsValidIfAreaIsOccupied)
{
    RobotStore::getInstance().addTeammate(Robot(2, RoleEnum::DEFENDER_ASSIST, geometry::Pose2D(0.0, -8.0, 0.0), geometry::Velocity2D(0.0, 0.0, 0.0)));

    EXPECT_TRUE(rule.isCurrentPositionValid());
}


TEST_F(inOppPenaltyArea, CurrentPositionIsInvalidAfterTimerElapsed)
{
    cEnvironmentField::getInstance().loadConfig("cEnvironment12x18");

    EXPECT_CALL(*timer_p, hasElapsed(_)).WillRepeatedly(Return(true));

    EXPECT_FALSE(rule.isCurrentPositionValid());
}

TEST_F(inOppPenaltyArea, CurrentPositionIsValidIfAreaIsEmpty)
{
    RobotStore::getInstance().addTeammate(Robot(2, RoleEnum::DEFENDER_ASSIST, geometry::Pose2D(0.0, 5.0, 0.0), geometry::Velocity2D(0.0, 0.0, 0.0)));

    EXPECT_TRUE(rule.isCurrentPositionValid());
}

TEST_F(inOppPenaltyArea, CurrentPositionIsInvalidIfAreaIsOccupied)
{
    cEnvironmentField::getInstance().loadConfig("cEnvironment12x18");

    RobotStore::getInstance().addTeammate(Robot(2, RoleEnum::DEFENDER_ASSIST, geometry::Pose2D(0.0, 8.0, 0.0), geometry::Velocity2D(0.0, 0.0, 0.0)));

    EXPECT_FALSE(rule.isCurrentPositionValid());
}

TEST_F(nearOppPenaltyArea, TargetPositionIsValidIfAreaEmpty)
{
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
    // since May 2019, Teamplay now always sets forbidden areas on (extended) goal area, which was previously done by pathPlanning
}

TEST_F(nearOppPenaltyArea, TargetPositionIsInvalidIfAreaIsOccupied)
{
    RobotStore::getInstance().addTeammate(Robot(2, RoleEnum::DEFENDER_ASSIST, geometry::Pose2D(0.0, 8.0, 0.0), geometry::Velocity2D(0.0, 0.0, 0.0)));
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
}

TEST_F(inOwnGoalArea, CurrentPositionIsValidForGoalie)
{
    RobotStore::getInstance().setOwnRobotRole(RoleEnum::GOALKEEPER);
    EXPECT_TRUE(rule.isCurrentPositionValid());
}

TEST_F(inOwnGoalArea, CurrentPositionIsInvalidForNonGoalie)
{
    cEnvironmentField::getInstance().loadConfig("cEnvironment12x18");

    RobotStore::getInstance().setOwnRobotRole(RoleEnum::DEFENDER_MAIN);
    EXPECT_FALSE(rule.isCurrentPositionValid());
}

TEST_F(inOppGoalArea, CurrentPositionIsInvalidForGoalie)
{
    cEnvironmentField::getInstance().loadConfig("cEnvironment12x18");

    RobotStore::getInstance().setOwnRobotRole(RoleEnum::GOALKEEPER);
    EXPECT_FALSE(rule.isCurrentPositionValid());
}

TEST_F(inOppGoalArea, CurrentPositionIsInvalidForNonGoalie)
{
    cEnvironmentField::getInstance().loadConfig("cEnvironment12x18");

    RobotStore::getInstance().setOwnRobotRole(RoleEnum::DEFENDER_MAIN);
    EXPECT_FALSE(rule.isCurrentPositionValid());
}

TEST_F(inOwnGoalArea, TargetPositionIsValidForGoalie)
{
    RobotStore::getInstance().setOwnRobotRole(RoleEnum::GOALKEEPER);
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
    // since May 2019, Teamplay now always sets forbidden areas on (extended) goal area, which was previously done by pathPlanning
    // in this case, it will not get a forbidden area on own goal area, but always on opponent area, although this is hardly relevant for goalie
}

TEST_F(inOwnGoalArea, TargetPositionIsInvalidForNonGoalie)
{
    RobotStore::getInstance().setOwnRobotRole(RoleEnum::DEFENDER_MAIN);
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
}

TEST_F(inOppGoalArea, TargetPositionIsInvalidForGoalie)
{
    RobotStore::getInstance().setOwnRobotRole(RoleEnum::GOALKEEPER);
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
}

TEST_F(inOppGoalArea, TargetPositionIsInvalidForNonGoalie)
{
    RobotStore::getInstance().setOwnRobotRole(RoleEnum::DEFENDER_MAIN);
    EXPECT_FALSE(rule.getForbiddenAreas().empty());
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
