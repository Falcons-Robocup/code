 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * robotStoreTest.cpp
 *
 *  Created on: Sep 16, 2017
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "teamplayTest.hpp"

/* SUT */
#include "int/stores/robotStore.hpp"

/* SUT dependencies */



/* Testing a store with one robot */

class ownRobotOnly : public TeamplayTest
{
public:
    ownRobotOnly()
    {
        robotStore::getInstance().clear();
        robotStore::getInstance().addOwnRobot(robot(2, treeEnum::DEFENDER_MAIN, Position2D( 1, -6, 0), Velocity2D()));
    }
};

TEST_F(ownRobotOnly, getAllRobotsReturnsOneRobot)
{
    EXPECT_EQ(1, robotStore::getInstance().getAllRobots().size());
    EXPECT_EQ( 1, robotStore::getInstance().getAllRobots().at(0).getPosition().x);
    EXPECT_EQ(-6, robotStore::getInstance().getAllRobots().at(0).getPosition().y);
    EXPECT_EQ(treeEnum::DEFENDER_MAIN, robotStore::getInstance().getAllRobots().at(0).getRole());
}

TEST_F(ownRobotOnly, getAllRobotsDoesIngoresChangesToRobots)
{
    robotStore::getInstance().getAllRobots().at(0).setRole(treeEnum::ATTACKER_ASSIST);
    robotStore::getInstance().getAllRobots().at(0).setPosition(Position2D( 2, -5, 0));

    EXPECT_EQ( 1, robotStore::getInstance().getAllRobots().at(0).getPosition().x);
    EXPECT_EQ(-6, robotStore::getInstance().getAllRobots().at(0).getPosition().y);
    EXPECT_EQ(treeEnum::DEFENDER_MAIN, robotStore::getInstance().getAllRobots().at(0).getRole());
}

TEST_F(ownRobotOnly, setRoleOfOwnRobot)
{
    robotStore::getInstance().setOwnRobotRole(treeEnum::ATTACKER_ASSIST);
    EXPECT_EQ(treeEnum::ATTACKER_ASSIST, robotStore::getInstance().getOwnRobot().getRole());

    robotStore::getInstance().setOwnRobotRole(treeEnum::DEFENDER_MAIN);
    EXPECT_EQ(treeEnum::DEFENDER_MAIN, robotStore::getInstance().getOwnRobot().getRole());
}

TEST_F(ownRobotOnly, ownAssistentIsAbsent)
{
    EXPECT_FALSE(robotStore::getInstance().getAssistantOfOwnRobot());
}

TEST_F(ownRobotOnly, noTeamMatesArePresent)
{
    EXPECT_EQ(0, robotStore::getInstance().getAllRobotsExclOwnRobot().size());
}



/* Testing a store with multiple robots */

class teamWithoutAssistants : public TeamplayTest
{
public:
    teamWithoutAssistants()
    {
        robotStore::getInstance().clear();
        robotStore::getInstance().addTeammate(robot(1, treeEnum::R_GOALKEEPER,    Position2D(-1, -8, 0), Velocity2D()));
        robotStore::getInstance().addTeammate(robot(2, treeEnum::DEFENDER_MAIN,   Position2D( 1, -6, 0), Velocity2D()));
        robotStore::getInstance().addOwnRobot(robot(8, treeEnum::ATTACKER_MAIN,   Position2D(-3,  6, 0), Velocity2D()));
    }
};

class completeTeam : public teamWithoutAssistants
{
public:
    completeTeam()
    {
        robotStore::getInstance().addTeammate(robot(3, treeEnum::DEFENDER_ASSIST, Position2D(-2, -3, 0), Velocity2D()));

        auto teammate_with_ball = robot(5, treeEnum::ATTACKER_ASSIST, Position2D( 2,  1, 0), Velocity2D());
        teammate_with_ball.claimsBallPossession();
        robotStore::getInstance().addTeammate(teammate_with_ball);
    }
};

TEST_F(completeTeam, StoreIsEmptyAfterClear)
{
    robotStore::getInstance().clear();
    EXPECT_TRUE(robotStore::getInstance().getAllRobots().empty());
}

TEST_F(completeTeam, getAllRobotsReturnsFiveRobots)
{
    EXPECT_EQ(5, robotStore::getInstance().getAllRobots().size());
}

TEST_F(completeTeam, getOwnRobotReturnsOwnRobot)
{
    EXPECT_EQ(8, robotStore::getInstance().getOwnRobot().getNumber());
    EXPECT_EQ(treeEnum::ATTACKER_MAIN, robotStore::getInstance().getOwnRobot().getRole());
    EXPECT_EQ(-3, robotStore::getInstance().getOwnRobot().getPosition().x);
    EXPECT_EQ( 6, robotStore::getInstance().getOwnRobot().getPosition().y);
}

TEST_F(completeTeam, testExchangeAndUndoExchange)
{
    EXPECT_EQ(8, robotStore::getInstance().getOwnRobot().getNumber());
    EXPECT_FALSE(robotStore::getInstance().getOwnRobot().hasBall());
    EXPECT_EQ(treeEnum::ATTACKER_MAIN, robotStore::getInstance().getOwnRobot().getRole());

    robotStore::getInstance().exchangeOwnRobotWith(5);

    EXPECT_EQ(5, robotStore::getInstance().getOwnRobot().getNumber());
    EXPECT_TRUE(robotStore::getInstance().getOwnRobot().hasBall());
    EXPECT_EQ(treeEnum::ATTACKER_ASSIST, robotStore::getInstance().getOwnRobot().getRole());

    robotStore::getInstance().undoExchange();

    EXPECT_EQ(8, robotStore::getInstance().getOwnRobot().getNumber());
    EXPECT_FALSE(robotStore::getInstance().getOwnRobot().hasBall());
    EXPECT_EQ(treeEnum::ATTACKER_MAIN, robotStore::getInstance().getOwnRobot().getRole());
}

TEST_F(completeTeam, undoExchangeWhileNoExchangeInProgressThrowsException)
{
    EXPECT_ANY_THROW(robotStore::getInstance().undoExchange());
}

TEST_F(completeTeam, exchangeWithUnknownRobotThrowsException)
{
    EXPECT_ANY_THROW(robotStore::getInstance().exchangeOwnRobotWith(13));
}

TEST_F(completeTeam, secondExchangeThrowsException)
{
    robotStore::getInstance().exchangeOwnRobotWith(1);
    EXPECT_ANY_THROW(robotStore::getInstance().exchangeOwnRobotWith(2));
    robotStore::getInstance().undoExchange();
}

TEST_F(completeTeam, setDuplicateRoleOfOwnRobot)
{
    robotStore::getInstance().setOwnRobotRole(treeEnum::R_GOALKEEPER);
    // Manual check: an error is printed
}

TEST_F(completeTeam, setMultipleRobotsToStopRole)
{
    robotStore::getInstance().setOwnRobotRole(treeEnum::R_ROBOT_STOP);
    robotStore::getInstance().exchangeOwnRobotWith(5);
    robotStore::getInstance().setOwnRobotRole(treeEnum::R_ROBOT_STOP);
    robotStore::getInstance().undoExchange();
}

TEST_F(completeTeam, ownAssistantIsPresentAndHasBall)
{
    ASSERT_NE(boost::none, robotStore::getInstance().getAssistantOfOwnRobot());
    EXPECT_TRUE(robotStore::getInstance().getAssistantOfOwnRobot()->hasBall());
}

TEST_F(completeTeam, onlyGoalkeeperInOwnPenaltyArea)
{
    EXPECT_EQ(0, robotStore::getInstance().getAllRobotsExclGoalieInArea(fieldArea::OWN_PENALTYAREA).size());
    EXPECT_EQ(1, robotStore::getInstance().getAllRobotsInArea(fieldArea::OWN_PENALTYAREA).size());
    EXPECT_EQ(1, robotStore::getInstance().getAllRobotsExclOwnRobotInArea(fieldArea::OWN_PENALTYAREA).size());
    EXPECT_EQ(treeEnum::R_GOALKEEPER, robotStore::getInstance().getAllRobotsInArea(fieldArea::OWN_PENALTYAREA).at(0).getRole());
    EXPECT_EQ(treeEnum::R_GOALKEEPER, robotStore::getInstance().getAllRobotsExclOwnRobotInArea(fieldArea::OWN_PENALTYAREA).at(0).getRole());
}

TEST_F(completeTeam, threeRobotsOnOwnHalf)
{
    EXPECT_EQ(3, robotStore::getInstance().getAllRobotsInArea(fieldArea::OWN_SIDE).size());

    // One of the robots is expected to be the goalie
    EXPECT_EQ(2, robotStore::getInstance().getAllRobotsExclGoalieInArea(fieldArea::OWN_SIDE).size());

    // The own robot is not at the own side
    EXPECT_EQ(3, robotStore::getInstance().getAllRobotsExclOwnRobotInArea(fieldArea::OWN_SIDE).size());
}

TEST_F(completeTeam, twoRobotsOnOpponentHalf)
{
    EXPECT_EQ(2, robotStore::getInstance().getAllRobotsInArea(fieldArea::OPP_SIDE).size());

    // None of the robots is expected to be the goalie
    EXPECT_EQ(2, robotStore::getInstance().getAllRobotsExclGoalieInArea(fieldArea::OPP_SIDE).size());

    // The own robot is at the opponent side
    EXPECT_EQ(1, robotStore::getInstance().getAllRobotsExclOwnRobotInArea(fieldArea::OPP_SIDE).size());
}

TEST_F(completeTeam, fourTeamMatesArePresent)
{
    EXPECT_EQ(4, robotStore::getInstance().getAllRobotsExclOwnRobot().size());
}

TEST_F(teamWithoutAssistants, fieldAssistentsAreNotPresent)
{
    EXPECT_FALSE(robotStore::getInstance().getAssistantOfRole(treeEnum::R_GOALKEEPER));
    EXPECT_FALSE(robotStore::getInstance().getAssistantOfRole(treeEnum::DEFENDER_MAIN));
    EXPECT_FALSE(robotStore::getInstance().getAssistantOfRole(treeEnum::ATTACKER_MAIN));
}

TEST_F(teamWithoutAssistants, defenderAssistentIsPresent)
{
    robotStore::getInstance().addTeammate(robot(4, treeEnum::DEFENDER_ASSIST, Position2D(), Velocity2D()));

    EXPECT_EQ(boost::none, robotStore::getInstance().getAssistantOfRole(treeEnum::R_GOALKEEPER));
    EXPECT_EQ(boost::none, robotStore::getInstance().getAssistantOfRole(treeEnum::ATTACKER_MAIN));

    EXPECT_EQ(treeEnum::DEFENDER_ASSIST, robotStore::getInstance().getAssistantOfRole(treeEnum::DEFENDER_MAIN)->getRole());
    EXPECT_EQ(treeEnum::DEFENDER_MAIN, robotStore::getInstance().getAssistantOfRole(treeEnum::DEFENDER_ASSIST)->getRole());
}

TEST_F(teamWithoutAssistants, attackerAssistentIsPresent)
{
    robotStore::getInstance().addTeammate(robot(5, treeEnum::ATTACKER_ASSIST, Position2D(), Velocity2D()));

    EXPECT_EQ(boost::none, robotStore::getInstance().getAssistantOfRole(treeEnum::R_GOALKEEPER));
    EXPECT_EQ(boost::none, robotStore::getInstance().getAssistantOfRole(treeEnum::DEFENDER_MAIN));

    EXPECT_EQ(treeEnum::ATTACKER_ASSIST, robotStore::getInstance().getAssistantOfRole(treeEnum::ATTACKER_MAIN)->getRole());
    EXPECT_EQ(treeEnum::ATTACKER_MAIN, robotStore::getInstance().getAssistantOfRole(treeEnum::ATTACKER_ASSIST)->getRole());
}

TEST_F(completeTeam, bothAssistentsArePresent)
{
    EXPECT_EQ(boost::none, robotStore::getInstance().getAssistantOfRole(treeEnum::R_GOALKEEPER));
    EXPECT_NE(boost::none, robotStore::getInstance().getAssistantOfRole(treeEnum::DEFENDER_MAIN));
    EXPECT_NE(boost::none, robotStore::getInstance().getAssistantOfRole(treeEnum::ATTACKER_MAIN));
    EXPECT_NE(boost::none, robotStore::getInstance().getAssistantOfRole(treeEnum::DEFENDER_ASSIST));
    EXPECT_NE(boost::none, robotStore::getInstance().getAssistantOfRole(treeEnum::ATTACKER_ASSIST));
}

TEST_F(completeTeam, getAllRobotsExclGoalie)
{
    EXPECT_EQ(4, robotStore::getInstance().getAllRobotsExclGoalie().size());
}

TEST_F(completeTeam, getAllRobotsExclGoalieSortedByDistanceToCenter)
{
    auto robots = robotStore::getInstance().getAllRobotsExclGoalieSortedByDistanceTo(Point2D(0, 0));

    ASSERT_EQ(4, robots.size());

    EXPECT_EQ(5, robots.at(0).getNumber());
    EXPECT_EQ(3, robots.at(1).getNumber());
    EXPECT_EQ(2, robots.at(2).getNumber());
    EXPECT_EQ(8, robots.at(3).getNumber());
}

TEST_F(completeTeam, getAllRobotsExclGoalieSortedByDistanceToCorner)
{
    auto robots = robotStore::getInstance().getAllRobotsExclGoalieSortedByDistanceTo(Point2D(-5.5, 8.5));

    ASSERT_EQ(4, robots.size());

    EXPECT_EQ(8, robots.at(0).getNumber());
    EXPECT_EQ(5, robots.at(1).getNumber());
    EXPECT_EQ(3, robots.at(2).getNumber());
    EXPECT_EQ(2, robots.at(3).getNumber());
}

TEST_F(completeTeam, getAllRobotsExclGoalieSortedByDistanceToOwnGoal)
{
    auto robots = robotStore::getInstance().getAllRobotsExclGoalieSortedByDistanceTo(Point2D(0.0, -8.5));

    ASSERT_EQ(4, robots.size());

    EXPECT_EQ(2, robots.at(0).getNumber());
    EXPECT_EQ(3, robots.at(1).getNumber());
    EXPECT_EQ(5, robots.at(2).getNumber());
    EXPECT_EQ(8, robots.at(3).getNumber());
}

TEST_F(completeTeam, getAllRobotsExclLowestIDSortedByDistanceToCenter)
{
    auto robots = robotStore::getInstance().getAllRobotsExclLowestIDSortedByDistanceTo(Point2D(0, 0));

    ASSERT_EQ(4, robots.size());

    EXPECT_EQ(5, robots.at(0).getNumber());
    EXPECT_EQ(3, robots.at(1).getNumber());
    EXPECT_EQ(2, robots.at(2).getNumber());
    EXPECT_EQ(8, robots.at(3).getNumber());
}

TEST_F(completeTeam, getAllRobotsExclLowestIDSortedByDistanceToCorner)
{
    auto robots = robotStore::getInstance().getAllRobotsExclLowestIDSortedByDistanceTo(Point2D(-5.5, 8.5));

    ASSERT_EQ(4, robots.size());

    EXPECT_EQ(8, robots.at(0).getNumber());
    EXPECT_EQ(5, robots.at(1).getNumber());
    EXPECT_EQ(3, robots.at(2).getNumber());
    EXPECT_EQ(2, robots.at(3).getNumber());
}

TEST_F(completeTeam, getAllRobotsExclLowestIDSortedByDistanceToOwnGoal)
{
    auto robots = robotStore::getInstance().getAllRobotsExclLowestIDSortedByDistanceTo(Point2D(0.0, -8.5));

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
        robotStore::getInstance().clear();
        robotStore::getInstance().addTeammate(robot(2, treeEnum::DEFENDER_MAIN, Position2D( 1, -6, 0), Velocity2D()));
    }
};

TEST_F(noOwnRobot, getOwnRobotThrowsException)
{
    EXPECT_ANY_THROW(robotStore::getInstance().getOwnRobot());
}



int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
