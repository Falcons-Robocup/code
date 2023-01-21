// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotStoreTest.cpp
 *
 *  Created on: Sep 16, 2017
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "../TeamplayTest.hpp"

/* SUT */
#include "int/stores/RobotStore.hpp"

/* SUT dependencies */
#include "cEnvironmentField.hpp"



/* Testing a store with one robot */

class ownRobotOnly : public TeamplayTest
{
public:
    ownRobotOnly()
    {
        RobotStore::getInstance().clear();
        RobotStore::getInstance().addOwnRobot(Robot(2, RoleEnum::DEFENDER_MAIN, geometry::Pose2D( 1, -6, 0), geometry::Velocity2D()));
    }
};

TEST_F(ownRobotOnly, getAllRobotsReturnsOneRobot)
{
    EXPECT_EQ(1, RobotStore::getInstance().getAllRobots().size());
    EXPECT_EQ( 1, RobotStore::getInstance().getAllRobots().at(0).getPosition().x);
    EXPECT_EQ(-6, RobotStore::getInstance().getAllRobots().at(0).getPosition().y);
    EXPECT_EQ(RoleEnum::DEFENDER_MAIN, RobotStore::getInstance().getAllRobots().at(0).getRole());
}

TEST_F(ownRobotOnly, getAllRobotsDoesIngoresChangesToRobots)
{
    RobotStore::getInstance().getAllRobots().at(0).setRole(RoleEnum::ATTACKER_ASSIST);
    RobotStore::getInstance().getAllRobots().at(0).setPosition(geometry::Pose2D( 2, -5, 0));

    EXPECT_EQ( 1, RobotStore::getInstance().getAllRobots().at(0).getPosition().x);
    EXPECT_EQ(-6, RobotStore::getInstance().getAllRobots().at(0).getPosition().y);
    EXPECT_EQ(RoleEnum::DEFENDER_MAIN, RobotStore::getInstance().getAllRobots().at(0).getRole());
}

TEST_F(ownRobotOnly, setRoleOfOwnRobot)
{
    RobotStore::getInstance().setOwnRobotRole(RoleEnum::ATTACKER_ASSIST);
    EXPECT_EQ(RoleEnum::ATTACKER_ASSIST, RobotStore::getInstance().getOwnRobot().getRole());

    RobotStore::getInstance().setOwnRobotRole(RoleEnum::DEFENDER_MAIN);
    EXPECT_EQ(RoleEnum::DEFENDER_MAIN, RobotStore::getInstance().getOwnRobot().getRole());
}

TEST_F(ownRobotOnly, ownAssistentIsAbsent)
{
    EXPECT_FALSE(RobotStore::getInstance().getAssistantOfOwnRobot());
}

TEST_F(ownRobotOnly, noTeamMatesArePresent)
{
    EXPECT_EQ(0, RobotStore::getInstance().getAllRobotsExclOwnRobot().size());
}



/* Testing a store with multiple robots */

class teamWithoutAssistants : public TeamplayTest
{
public:
    teamWithoutAssistants()
    {
        RobotStore::getInstance().clear();
        RobotStore::getInstance().addTeammate(Robot(1, RoleEnum::GOALKEEPER,      geometry::Pose2D(-1, -8, 0), geometry::Velocity2D()));
        RobotStore::getInstance().addTeammate(Robot(2, RoleEnum::DEFENDER_MAIN,   geometry::Pose2D( 1, -6, 0), geometry::Velocity2D()));
        RobotStore::getInstance().addOwnRobot(Robot(8, RoleEnum::ATTACKER_MAIN,   geometry::Pose2D(-3,  6, 0), geometry::Velocity2D()));
    }
};

class completeTeam : public teamWithoutAssistants
{
public:
    completeTeam()
    {
        RobotStore::getInstance().addTeammate(Robot(3, RoleEnum::DEFENDER_ASSIST, geometry::Pose2D(-2, -3, 0), geometry::Velocity2D()));

        auto teammate_with_ball = Robot(5, RoleEnum::ATTACKER_ASSIST, geometry::Pose2D( 2,  1, 0), geometry::Velocity2D());
        teammate_with_ball.claimsBallPossession();
        RobotStore::getInstance().addTeammate(teammate_with_ball);
    }
};

TEST_F(completeTeam, StoreIsEmptyAfterClear)
{
    RobotStore::getInstance().clear();
    EXPECT_TRUE(RobotStore::getInstance().getAllRobots().empty());
}

TEST_F(completeTeam, getAllRobotsReturnsFiveRobots)
{
    EXPECT_EQ(5, RobotStore::getInstance().getAllRobots().size());
}

TEST_F(completeTeam, getOwnRobotReturnsOwnRobot)
{
    EXPECT_EQ(8, RobotStore::getInstance().getOwnRobot().getNumber());
    EXPECT_EQ(RoleEnum::ATTACKER_MAIN, RobotStore::getInstance().getOwnRobot().getRole());
    EXPECT_EQ(-3, RobotStore::getInstance().getOwnRobot().getPosition().x);
    EXPECT_EQ( 6, RobotStore::getInstance().getOwnRobot().getPosition().y);
}

TEST_F(completeTeam, testExchangeAndUndoExchange)
{
    EXPECT_EQ(8, RobotStore::getInstance().getOwnRobot().getNumber());
    EXPECT_FALSE(RobotStore::getInstance().getOwnRobot().hasBall());
    EXPECT_EQ(RoleEnum::ATTACKER_MAIN, RobotStore::getInstance().getOwnRobot().getRole());

    RobotStore::getInstance().exchangeOwnRobotWith(5);

    EXPECT_EQ(5, RobotStore::getInstance().getOwnRobot().getNumber());
    EXPECT_TRUE(RobotStore::getInstance().getOwnRobot().hasBall());
    EXPECT_EQ(RoleEnum::ATTACKER_ASSIST, RobotStore::getInstance().getOwnRobot().getRole());

    RobotStore::getInstance().undoExchange();

    EXPECT_EQ(8, RobotStore::getInstance().getOwnRobot().getNumber());
    EXPECT_FALSE(RobotStore::getInstance().getOwnRobot().hasBall());
    EXPECT_EQ(RoleEnum::ATTACKER_MAIN, RobotStore::getInstance().getOwnRobot().getRole());
}

TEST_F(completeTeam, undoExchangeWhileNoExchangeInProgressThrowsException)
{
    EXPECT_ANY_THROW(RobotStore::getInstance().undoExchange());
}

TEST_F(completeTeam, exchangeWithUnknownRobotThrowsException)
{
    EXPECT_ANY_THROW(RobotStore::getInstance().exchangeOwnRobotWith(13));
}

TEST_F(completeTeam, secondExchangeThrowsException)
{
    RobotStore::getInstance().exchangeOwnRobotWith(1);
    EXPECT_ANY_THROW(RobotStore::getInstance().exchangeOwnRobotWith(2));
    RobotStore::getInstance().undoExchange();
}

TEST_F(completeTeam, setDuplicateRoleOfOwnRobot)
{
    RobotStore::getInstance().setOwnRobotRole(RoleEnum::GOALKEEPER);
    // Manual check: an error is printed
}

TEST_F(completeTeam, setMultipleRobotsToStopRole)
{
    RobotStore::getInstance().setOwnRobotRole(RoleEnum::STOP);
    RobotStore::getInstance().exchangeOwnRobotWith(5);
    RobotStore::getInstance().setOwnRobotRole(RoleEnum::STOP);
    RobotStore::getInstance().undoExchange();
}

TEST_F(completeTeam, ownAssistantIsPresentAndHasBall)
{
    ASSERT_NE(boost::none, RobotStore::getInstance().getAssistantOfOwnRobot());
    EXPECT_TRUE(RobotStore::getInstance().getAssistantOfOwnRobot()->hasBall());
}

TEST_F(completeTeam, onlyGoalkeeperInOwnPenaltyArea)
{
    cEnvironmentField::getInstance().loadConfig("cEnvironment12x18");

    EXPECT_EQ(0, RobotStore::getInstance().getAllRobotsExclGoalieInArea(FieldArea::OWN_PENALTYAREA).size());
    EXPECT_EQ(1, RobotStore::getInstance().getAllRobotsInArea(FieldArea::OWN_PENALTYAREA).size());
    EXPECT_EQ(1, RobotStore::getInstance().getAllRobotsExclOwnRobotInArea(FieldArea::OWN_PENALTYAREA).size());
    EXPECT_EQ(RoleEnum::GOALKEEPER, RobotStore::getInstance().getAllRobotsInArea(FieldArea::OWN_PENALTYAREA).at(0).getRole());
    EXPECT_EQ(RoleEnum::GOALKEEPER, RobotStore::getInstance().getAllRobotsExclOwnRobotInArea(FieldArea::OWN_PENALTYAREA).at(0).getRole());
}

TEST_F(completeTeam, threeRobotsOnOwnHalf)
{
    EXPECT_EQ(3, RobotStore::getInstance().getAllRobotsInArea(FieldArea::OWN_SIDE).size());

    // One of the robots is expected to be the goalie
    EXPECT_EQ(2, RobotStore::getInstance().getAllRobotsExclGoalieInArea(FieldArea::OWN_SIDE).size());

    // The own robot is not at the own side
    EXPECT_EQ(3, RobotStore::getInstance().getAllRobotsExclOwnRobotInArea(FieldArea::OWN_SIDE).size());
}

TEST_F(completeTeam, twoRobotsOnOpponentHalf)
{
    EXPECT_EQ(2, RobotStore::getInstance().getAllRobotsInArea(FieldArea::OPP_SIDE).size());

    // None of the robots is expected to be the goalie
    EXPECT_EQ(2, RobotStore::getInstance().getAllRobotsExclGoalieInArea(FieldArea::OPP_SIDE).size());

    // The own robot is at the opponent side
    EXPECT_EQ(1, RobotStore::getInstance().getAllRobotsExclOwnRobotInArea(FieldArea::OPP_SIDE).size());
}

TEST_F(completeTeam, fourTeamMatesArePresent)
{
    EXPECT_EQ(4, RobotStore::getInstance().getAllRobotsExclOwnRobot().size());
}

TEST_F(teamWithoutAssistants, fieldAssistentsAreNotPresent)
{
    EXPECT_FALSE(RobotStore::getInstance().getAssistantOfRole(RoleEnum::GOALKEEPER));
    EXPECT_FALSE(RobotStore::getInstance().getAssistantOfRole(RoleEnum::DEFENDER_MAIN));
    EXPECT_FALSE(RobotStore::getInstance().getAssistantOfRole(RoleEnum::ATTACKER_MAIN));
}

TEST_F(teamWithoutAssistants, defenderAssistentIsPresent)
{
    RobotStore::getInstance().addTeammate(Robot(4, RoleEnum::DEFENDER_ASSIST, geometry::Pose2D(), geometry::Velocity2D()));

    EXPECT_EQ(boost::none, RobotStore::getInstance().getAssistantOfRole(RoleEnum::GOALKEEPER));
    EXPECT_EQ(boost::none, RobotStore::getInstance().getAssistantOfRole(RoleEnum::ATTACKER_MAIN));

    EXPECT_EQ(RoleEnum::DEFENDER_ASSIST, RobotStore::getInstance().getAssistantOfRole(RoleEnum::DEFENDER_MAIN)->getRole());
    EXPECT_EQ(RoleEnum::DEFENDER_MAIN, RobotStore::getInstance().getAssistantOfRole(RoleEnum::DEFENDER_ASSIST)->getRole());
}

TEST_F(teamWithoutAssistants, attackerAssistentIsPresent)
{
    RobotStore::getInstance().addTeammate(Robot(5, RoleEnum::ATTACKER_ASSIST, geometry::Pose2D(), geometry::Velocity2D()));

    EXPECT_EQ(boost::none, RobotStore::getInstance().getAssistantOfRole(RoleEnum::GOALKEEPER));
    EXPECT_EQ(boost::none, RobotStore::getInstance().getAssistantOfRole(RoleEnum::DEFENDER_MAIN));

    EXPECT_EQ(RoleEnum::ATTACKER_ASSIST, RobotStore::getInstance().getAssistantOfRole(RoleEnum::ATTACKER_MAIN)->getRole());
    EXPECT_EQ(RoleEnum::ATTACKER_MAIN, RobotStore::getInstance().getAssistantOfRole(RoleEnum::ATTACKER_ASSIST)->getRole());
}

TEST_F(completeTeam, bothAssistentsArePresent)
{
    EXPECT_EQ(boost::none, RobotStore::getInstance().getAssistantOfRole(RoleEnum::GOALKEEPER));
    EXPECT_NE(boost::none, RobotStore::getInstance().getAssistantOfRole(RoleEnum::DEFENDER_MAIN));
    EXPECT_NE(boost::none, RobotStore::getInstance().getAssistantOfRole(RoleEnum::ATTACKER_MAIN));
    EXPECT_NE(boost::none, RobotStore::getInstance().getAssistantOfRole(RoleEnum::DEFENDER_ASSIST));
    EXPECT_NE(boost::none, RobotStore::getInstance().getAssistantOfRole(RoleEnum::ATTACKER_ASSIST));
}

TEST_F(completeTeam, getAllRobotsExclGoalie)
{
    EXPECT_EQ(4, RobotStore::getInstance().getAllRobotsExclGoalie().size());
}

TEST_F(completeTeam, getAllRobotsExclGoalieSortedByDistanceToCenter)
{
    auto robots = RobotStore::getInstance().getAllRobotsExclGoalieSortedByDistanceTo(Point2D(0, 0));

    ASSERT_EQ(4, robots.size());

    EXPECT_EQ(5, robots.at(0).getNumber());
    EXPECT_EQ(3, robots.at(1).getNumber());
    EXPECT_EQ(2, robots.at(2).getNumber());
    EXPECT_EQ(8, robots.at(3).getNumber());
}

TEST_F(completeTeam, getAllRobotsExclGoalieSortedByDistanceToCorner)
{
    auto robots = RobotStore::getInstance().getAllRobotsExclGoalieSortedByDistanceTo(Point2D(-5.5, 8.5));

    ASSERT_EQ(4, robots.size());

    EXPECT_EQ(8, robots.at(0).getNumber());
    EXPECT_EQ(5, robots.at(1).getNumber());
    EXPECT_EQ(3, robots.at(2).getNumber());
    EXPECT_EQ(2, robots.at(3).getNumber());
}

TEST_F(completeTeam, getAllRobotsExclGoalieSortedByDistanceToOwnGoal)
{
    auto robots = RobotStore::getInstance().getAllRobotsExclGoalieSortedByDistanceTo(Point2D(0.0, -8.5));

    ASSERT_EQ(4, robots.size());

    EXPECT_EQ(2, robots.at(0).getNumber());
    EXPECT_EQ(3, robots.at(1).getNumber());
    EXPECT_EQ(5, robots.at(2).getNumber());
    EXPECT_EQ(8, robots.at(3).getNumber());
}

TEST_F(completeTeam, getAllRobotsExclLowestIDSortedByDistanceToCenter)
{
    auto robots = RobotStore::getInstance().getAllRobotsExclLowestIDSortedByDistanceTo(Point2D(0, 0));

    ASSERT_EQ(4, robots.size());

    EXPECT_EQ(5, robots.at(0).getNumber());
    EXPECT_EQ(3, robots.at(1).getNumber());
    EXPECT_EQ(2, robots.at(2).getNumber());
    EXPECT_EQ(8, robots.at(3).getNumber());
}

TEST_F(completeTeam, getAllRobotsExclLowestIDSortedByDistanceToCorner)
{
    auto robots = RobotStore::getInstance().getAllRobotsExclLowestIDSortedByDistanceTo(Point2D(-5.5, 8.5));

    ASSERT_EQ(4, robots.size());

    EXPECT_EQ(8, robots.at(0).getNumber());
    EXPECT_EQ(5, robots.at(1).getNumber());
    EXPECT_EQ(3, robots.at(2).getNumber());
    EXPECT_EQ(2, robots.at(3).getNumber());
}

TEST_F(completeTeam, getAllRobotsExclLowestIDSortedByDistanceToOwnGoal)
{
    auto robots = RobotStore::getInstance().getAllRobotsExclLowestIDSortedByDistanceTo(Point2D(0.0, -8.5));

    ASSERT_EQ(4, robots.size());

    EXPECT_EQ(2, robots.at(0).getNumber());
    EXPECT_EQ(3, robots.at(1).getNumber());
    EXPECT_EQ(5, robots.at(2).getNumber());
    EXPECT_EQ(8, robots.at(3).getNumber());
}


/* Exceptional tests: testing a store without own robot */

class noOwnRobot : public TeamplayTest
{
public:
    noOwnRobot()
    {
        RobotStore::getInstance().clear();
        RobotStore::getInstance().addTeammate(Robot(2, RoleEnum::DEFENDER_MAIN, geometry::Pose2D( 1, -6, 0), geometry::Velocity2D()));
    }
};

TEST_F(noOwnRobot, getOwnRobotThrowsException)
{
    EXPECT_ANY_THROW(RobotStore::getInstance().getOwnRobot());
}



int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
