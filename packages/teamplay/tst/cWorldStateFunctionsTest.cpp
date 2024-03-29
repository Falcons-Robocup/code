// Copyright 2015-2021 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cWorldStateFunctionsTest.cpp
 *
 *  Created on: Oct 13, 2015
 *      Author: Ivo Matthijssen
 */
// Bring in gtest
#include "gtest/gtest.h"

/* Include trace utility */
#include "tracing.hpp"

// Bring in my package's API, which is what I'm testing
#include "int/cWorldStateFunctions.hpp"
#include "int/stores/ballStore.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/stores/fieldDimensionsStore.hpp"
#include "int/stores/gameStateStore.hpp"
#include "int/stores/obstacleStore.hpp"
#include "int/stores/robotStore.hpp"
#include "int/actions/cAbstractAction.hpp"

#include <math.h>

using namespace ::testing;
using namespace teamplay;


// ======================
// doesTeamHaveBall
// ======================

TEST(WorldStateFunctions, doesTeamHaveBall_WhenBallPossessionThisRobot_ShouldReturnTrue)
{
    // Arrange
    teamplay::robotStore::getInstance().clear();

    // Update administration that own robot has the ball
    auto own_robot = teamplay::robot();
    own_robot.claimsBallPossession();
    teamplay::robotStore::getInstance().addOwnRobot(own_robot);

    auto teammate = teamplay::robot();
    teammate.losesBallPossession();
    teamplay::robotStore::getInstance().addTeammate(teammate);

    // Act
    std::map<std::string, std::string> params;
    bool result = doesTeamHaveBall(params);

    // Assert
    EXPECT_TRUE(result);
}

TEST(WorldStateFunctions, doesTeamHaveBall_WhenBallPossessionOtherTeamRobot_ShouldReturnTrue)
{
    // Arrange
    teamplay::robotStore::getInstance().clear();

    // Update administration that own robot does *not* have the ball
    auto own_robot = teamplay::robot();
    own_robot.losesBallPossession();
    teamplay::robotStore::getInstance().addOwnRobot(own_robot);

    auto teammate = teamplay::robot();
    teammate.claimsBallPossession();
    teamplay::robotStore::getInstance().addTeammate(teammate);

    // Act
    std::map<std::string, std::string> params;
    bool result = doesTeamHaveBall(params);

    // Assert
    EXPECT_TRUE(result);
}

TEST(WorldStateFunctions, doesTeamHaveBall_WhenFieldHasBall_ShouldReturnFalse)
{
    // Arrange
    teamplay::robotStore::getInstance().clear();

    // Update administration that field has the ball
    auto own_robot = teamplay::robot();
    own_robot.losesBallPossession();
    teamplay::robotStore::getInstance().addOwnRobot(own_robot);

    auto teammate = teamplay::robot();
    teammate.losesBallPossession();
    teamplay::robotStore::getInstance().addTeammate(teammate);

    // Act
    std::map<std::string, std::string> params;
    bool result = doesTeamHaveBall(params);

    // Assert
    EXPECT_FALSE(result);
}

// ======================
// doesOwnRobotHaveBall
// ======================

TEST(WorldStateFunctions, doesOwnRobotHaveBall_WhenBallPossessionThisRobot_ShouldReturnTrue)
{
    // Arrange
    teamplay::robotStore::getInstance().clear();

    // Update administration that own robot has the ball
    auto own_robot = teamplay::robot();
    own_robot.claimsBallPossession();
    teamplay::robotStore::getInstance().addOwnRobot(own_robot);

    auto teammate = teamplay::robot();
    teammate.losesBallPossession();
    teamplay::robotStore::getInstance().addTeammate(teammate);

    // Act
    std::map<std::string, std::string> params;
    bool result = doesOwnRobotHaveBall(params);

    // Assert
    EXPECT_TRUE(result);
}

TEST(WorldStateFunctions, doesOwnRobotHaveBall_WhenBallPossessionOtherTeamRobot_ShouldReturnFalse)
{
    // Arrange
    teamplay::robotStore::getInstance().clear();

    // Update administration that other team robot has the ball
    auto own_robot = teamplay::robot();
    own_robot.losesBallPossession();
    teamplay::robotStore::getInstance().addOwnRobot(own_robot);

    auto teammate = teamplay::robot();
    teammate.claimsBallPossession();
    teamplay::robotStore::getInstance().addTeammate(teammate);

    // Act
    std::map<std::string, std::string> params;
    bool result = doesOwnRobotHaveBall(params);

    // Assert
    EXPECT_FALSE(result);
}

// ======================
// isBallInOwnPenaltyArea
// ======================

TEST(WorldStateFunctions, isBallInOwnPenaltyArea_WhenBallLocationInOwnHalf_ShouldReturnTrue)
{
    // Arrange
    // Update WorldModel administration that the ball is in the penalty area
    Area2D penaltyArea = teamplay::fieldDimensionsStore::getFieldDimensions().getArea(teamplay::fieldArea::OWN_PENALTYAREA);
    auto ball_pos_x = (penaltyArea.ll.x + penaltyArea.ur.x) / 2.0;
    auto ball_pos_y = (penaltyArea.ll.y + penaltyArea.ur.y) / 2.0;

    teamplay::ballStore::getBall().setPosition(Point3D(ball_pos_x, ball_pos_y, 0.0));

    // Act
    std::map<std::string, std::string> params;
    bool result = isBallInOwnPenaltyArea(params);

    // Assert
    EXPECT_TRUE(result);
}

TEST(WorldStateFunctions, isBallInOwnPenaltyArea_WhenBallLocationInOpponentHalf_ShouldReturnFalse)
{
    // Arrange
    // Update WorldModel administration that the ball is in the penalty area
    Area2D penaltyArea = teamplay::fieldDimensionsStore::getFieldDimensions().getArea(teamplay::fieldArea::OPP_PENALTYAREA);
    auto ball_pos_x = (penaltyArea.ll.x + penaltyArea.ur.x) / 2.0;
    auto ball_pos_y = (penaltyArea.ll.y + penaltyArea.ur.y) / 2.0;

    teamplay::ballStore::getBall().setPosition(Point3D(ball_pos_x, ball_pos_y, 0.0));

    // Act
    std::map<std::string, std::string> params;
    bool result = isBallInOwnPenaltyArea(params);

    // Assert
    EXPECT_FALSE(result);
}

TEST(WorldStateFunctions, isBallInOwnPenaltyArea_WhenNoBallSeen_ShouldReturnFalse)
{
    // Arrange
    teamplay::ballStore::getBall().setPosition(Point3D(1.0, 1.0, 0.0));
    teamplay::ballStore::getBall().setPositionUnknown();

    // Act
    std::map<std::string, std::string> params;
    bool result = isBallInOwnPenaltyArea(params);

    // Assert
    EXPECT_FALSE(result);
}

// ======================
// isBallInOwnPenaltyArea
// ======================

TEST(WorldStateFunctions, isBallInOpponentPenaltyArea_WhenBallLocationInOpponentHalf_ShouldReturnTrue)
{
    // Arrange
    // Update WorldModel administration that the ball is in the penalty area
    Area2D penaltyArea = teamplay::fieldDimensionsStore::getFieldDimensions().getArea(teamplay::fieldArea::OPP_PENALTYAREA);
    auto ball_pos_x = (penaltyArea.ll.x + penaltyArea.ur.x) / 2.0;
    auto ball_pos_y = (penaltyArea.ll.y + penaltyArea.ur.y) / 2.0;

    teamplay::ballStore::getBall().setPosition(Point3D(ball_pos_x, ball_pos_y, 0.0));

    // Act
    std::map<std::string, std::string> params;
    bool result = isBallInOpponentPenaltyArea(params);

    // Assert
    EXPECT_TRUE(result);
}

TEST(WorldStateFunctions, isBallInOpponentPenaltyArea_WhenBallLocationInOwnHalf_ShouldReturnFalse)
{
    // Arrange
    // Update WorldModel administration that the ball is in the penalty area
    Area2D penaltyArea = teamplay::fieldDimensionsStore::getFieldDimensions().getArea(teamplay::fieldArea::OWN_PENALTYAREA);
    auto ball_pos_x = (penaltyArea.ll.x + penaltyArea.ur.x) / 2.0;
    auto ball_pos_y = (penaltyArea.ll.y + penaltyArea.ur.y) / 2.0;

    teamplay::ballStore::getBall().setPosition(Point3D(ball_pos_x, ball_pos_y, 0.0));

    // Act
    std::map<std::string, std::string> params;
    bool result = isBallInOpponentPenaltyArea(params);

    // Assert
    EXPECT_FALSE(result);
}

TEST(WorldStateFunctions, isBallInOpponentPenaltyArea_WhenNoBallSeen_ShouldReturnFalse)
{
    // Arrange
    teamplay::ballStore::getBall().setPosition(Point3D(1.0, 1.0, 0.0));
    teamplay::ballStore::getBall().setPositionUnknown();

    // Act
    std::map<std::string, std::string> params;
    bool result = isBallInOpponentPenaltyArea(params);

    // Assert
    EXPECT_FALSE(result);
}

// ======================
// getClosestTeammember
// ======================

TEST(WorldStateFunctions, getClosestTeammember_WhenNoRobotSet_ShouldThrow)
{
    // Arrange
    teamplay::robotStore::getInstance().clear();

    // Act
    EXPECT_ANY_THROW(cWorldStateFunctions::getInstance().getClosestTeammember(false));
}


TEST(WorldStateFunctions, getClosestTeammember_WhenNoTeammembersPresent_ShouldReturnEmptyOptional)
{
    // Arrange
    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot());

    // Act
    auto result = cWorldStateFunctions::getInstance().getClosestTeammember(false);

    // Assert
    EXPECT_FALSE((bool)result);
}

// Test fixture that sets up a second team member with id 8 and position x=6.0, y=7.0
// Sets own position at x=1.0,y=2.0
class WorldStateFunctions_WhenOneOtherRobotPresent : public ::testing::Test {
protected:

    WorldStateFunctions_WhenOneOtherRobotPresent()
    {
        // Start with a clean robot store
        teamplay::robotStore::getInstance().clear();

        // Define own robot
        teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(1.0, 2.0, 0.0), Velocity2D()));

        // Define one teammate with different id, role and location
        teamplay::robotStore::getInstance().addTeammate(teamplay::robot(8, treeEnum::DEFENDER_MAIN, Position2D(6.0, 7.0, 0.0), Velocity2D()));
    }
};

TEST_F(WorldStateFunctions_WhenOneOtherRobotPresent, getClosestTeammember_ShouldReturnValidOptional)
{
    // Arrange
    // Done in test fixture

    // Act
    auto result = cWorldStateFunctions::getInstance().getClosestTeammember(false);

    // Assert
    EXPECT_TRUE((bool)result);
}

TEST_F(WorldStateFunctions_WhenOneOtherRobotPresent, getClosestTeammember_ReturnClosestRobot)
{
    // Arrange
    // Done in test fixture

    // Act
    auto result = cWorldStateFunctions::getInstance().getClosestTeammember(false);

    // Assert
    ASSERT_TRUE((bool)result);
    EXPECT_EQ(6.0, result->getLocation().x) << "Wrong x position";
    EXPECT_EQ(7.0, result->getLocation().y) << "Wrong y position";
}

// Test fixture that sets up three other team members: 8 (x=6.0, y=7.0), 15 (x=1.0, y=3.0) and 20 (x=7.0, y=6.0)
// Sets own position at x=1.0,y=1.0
class WorldStateFunctions_WhenMultipleOtherRobotsPresent : public ::testing::Test {
protected:

    WorldStateFunctions_WhenMultipleOtherRobotsPresent()
{
        // Start with a clean robot store
        teamplay::robotStore::getInstance().clear();

        // Define own robot
        teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(1.0, 1.0, 0.0), Velocity2D()));

        // Define three teammates with different ids, roles and locations
        teamplay::robotStore::getInstance().addTeammate(teamplay::robot(8,  treeEnum::DEFENDER_MAIN,   Position2D(6.0, 7.0, 0.0), Velocity2D()));
        teamplay::robotStore::getInstance().addTeammate(teamplay::robot(15, treeEnum::DEFENDER_ASSIST, Position2D(1.0, 3.0, 0.0), Velocity2D()));
        teamplay::robotStore::getInstance().addTeammate(teamplay::robot(20, treeEnum::ATTACKER_ASSIST, Position2D(7.0, 6.0, 0.0), Velocity2D()));
}
};

TEST_F(WorldStateFunctions_WhenMultipleOtherRobotsPresent, getClosestTeammember_ShouldReturnTrue)
{
    // Arrange
    // Done in test fixture

    // Act
    auto result = cWorldStateFunctions::getInstance().getClosestTeammember(false);

    // Assert
    EXPECT_TRUE((bool)result);
}

TEST_F(WorldStateFunctions_WhenMultipleOtherRobotsPresent, getClosestTeammember_ReturnClosestRobot)
{
    // Arrange
    // Done in test fixture

    // Act
    auto result = cWorldStateFunctions::getInstance().getClosestTeammember(false);

    // Assert
    ASSERT_TRUE((bool)result);
    EXPECT_EQ(1.0, result->getLocation().x) << "Wrong x position";
    EXPECT_EQ(3.0, result->getLocation().y) << "Wrong y position";
}

// Test fixture that sets up one other team member at coordinate x=1.0,y=1.0
// Sets own position at x=1.0,y=1.0
class WorldStateFunctions_WhenTeamMemberAtExactSameLocation : public ::testing::Test {
protected:

    WorldStateFunctions_WhenTeamMemberAtExactSameLocation()
    {
        // Start with a clean robot store
        teamplay::robotStore::getInstance().clear();

        // Define own robot
        teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(1.0, 1.0, 0.0), Velocity2D()));

        // Define one teammate with different ids and role but same location
        teamplay::robotStore::getInstance().addTeammate(teamplay::robot(8,  treeEnum::DEFENDER_MAIN,   Position2D(1.0, 1.0, 0.0), Velocity2D()));
    }
};

TEST_F(WorldStateFunctions_WhenTeamMemberAtExactSameLocation, getClosestTeammember_ShouldReturnTrue)
{
    // Arrange
    // Done in test fixture

    // Act
    auto result = cWorldStateFunctions::getInstance().getClosestTeammember(false);

    // Assert
    EXPECT_TRUE((bool)result);
}

TEST_F(WorldStateFunctions_WhenTeamMemberAtExactSameLocation, getClosestTeammember_ReturnClosestRobot)
{
    // Arrange
    // Done in test fixture

    // Act
    auto result = cWorldStateFunctions::getInstance().getClosestTeammember(false);

    // Assert
    ASSERT_TRUE((bool)result);
    EXPECT_EQ(1.0, result->getLocation().x) << "Wrong x position";
    EXPECT_EQ(1.0, result->getLocation().y) << "Wrong y position";
}

// ======================
// getClosestOpponent
// ======================

TEST(TestSuiteWorldStateFunctionsGetClosestOpponent, closestOpponent_test1)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: OWN @ 5,5   ,  Bot8 @ 6,6, Bot 15 @ 8,8, Bot 20 @ 5.1,5.4  => Bot20 is closest

    // set my location and stuff
    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(5.0, 5.0, 0.0), Velocity2D()));

    // set other bot location(s) and stuff
    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(6.0, 6.0, 0.0)));
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(8.0, 8.0, 0.0)));
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(5.1, 5.4, 0.0)));

    // Execution
    float x,y;
    bool retval = cWorldStateFunctions::getInstance().getClosestOpponent(x,y);

    // Verification
    EXPECT_TRUE(retval == true);
    EXPECT_FLOAT_EQ( 5.1f, x);
    EXPECT_FLOAT_EQ( 5.4f, y);

}

TEST(TestSuiteWorldStateFunctionsMulptipleOpponentsOnOwnHalf, MultipleOpponentsOnOwnHalf_test1)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: Bot1 @ 0,1, Bot 2 @ 0,-1, => result shoud be false, not more than 1 opponent on own half

    // set other bot location(s) and stuff
    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(0.0, 1.0, 0.0)));
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(0.0, -1.0, 0.0)));

    // Execution
    std::map<std::string, std::string> params;
    bool retval = multipleOpponentsOnOwnHalf(params);

    // Verification
    EXPECT_TRUE(retval == false);
}

TEST(TestSuiteWorldStateFunctionsMulptipleOpponentsOnOwnHalf, MultipleOpponentsOnOwnHalf_test2)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: Bot1 @ 0,-1, Bot 2 @ 0,-2, => result shoud be true, more than 1 opponent on own half

    // set other bot location(s) and stuff
    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(0.0, -1.0, 0.0)));
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(0.0, -2.0, 0.0)));

    // Execution
    std::map<std::string, std::string> params;
    bool retval = multipleOpponentsOnOwnHalf(params);

    // Verification
    EXPECT_TRUE(retval == true);
}

TEST(TestSuiteWorldStateFunctionsIsAnAttackerOnOppHalf, IsAnAttackerOnOppHalf_test1)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: AttMain@ 0,2 => result shoud be true, an attacker on opponent half

    //
    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::DEFENDER_MAIN, Position2D(0.0, -2.0, 0.0), Velocity2D()));
    teamplay::robotStore::getInstance().addTeammate(teamplay::robot(2, treeEnum::ATTACKER_MAIN, Position2D(0.0, 2.0, 0.0), Velocity2D()));

    // Execution
    std::map<std::string, std::string> params;
    bool retval = isAnAttackerOnOppHalf(params);

    // Verification
    EXPECT_TRUE(retval == true);
}

TEST(TestSuiteWorldStateFunctionsIsAnAttackerOnOppHalf, IsAnAttackerOnOppHalf_test2)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: AttMain@ 0,-2 => result shoud be false, no attacker on opponent half

    //
    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::DEFENDER_MAIN, Position2D(0.0, -2.0, 0.0), Velocity2D()));
    teamplay::robotStore::getInstance().addTeammate(teamplay::robot(2, treeEnum::ATTACKER_MAIN, Position2D(0.0, -4.0, 0.0), Velocity2D()));

    // Execution
    std::map<std::string, std::string> params;
    bool retval = isAnAttackerOnOppHalf(params);

    // Verification
    EXPECT_FALSE(retval == true);
}

TEST(TestSuiteWorldStateFunctionsGetPotentialOpponentAttacker, PotentialOpponentAttacker_test1)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: Bot1 @ 1,-6, Bot2 @ -1,-7, Ball @ -1.1,-7.0  => Bot2 closest to goal but also to ball

    // set ball location
    teamplay::ballStore::getBall().setPosition(Point3D(-1.1, -7.0, 0.0));

    // set other bot location(s) and stuff
    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(1.0, -6.0, 0.0)));
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(-1.0, -7.0, 0.0)));

    // Execution
    float x,y;
    bool retval = cWorldStateFunctions::getInstance().getPotentialOpponentAttacker(x,y);
    std::map<std::string, std::string> params;
    bool retval2 = isPotentialOppAttackerPresent(params);

    // Verification
    EXPECT_TRUE(retval == true);
    EXPECT_FLOAT_EQ( 1.0f, x);
    EXPECT_FLOAT_EQ( -6.0f, y);
    EXPECT_TRUE(retval2 == true);
}

TEST(TestSuiteWorldStateFunctionsGetPotentialOpponentAttacker, PotentialOpponentAttacker_test2)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: Bot1 @ 1.0,-6.0, Bot2 @ -1.0,-7.0, Ball @ 1.1,-6.0  => Bot2 closest to goal but also to ball

    // set ball location
    teamplay::ballStore::getBall().setPosition(Point3D(1.1, -6.0, 0.0));

    // set other bot location(s) and stuff
    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(1.0, -6.0, 0.0)));
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(-1.0, -7.0, 0.0)));

    // Execution
    float x,y;
    bool retval = cWorldStateFunctions::getInstance().getPotentialOpponentAttacker(x,y);
    std::map<std::string, std::string> params;
    bool retval2 = isPotentialOppAttackerPresent(params);

    // Verification
    EXPECT_TRUE(retval == true);
    EXPECT_FLOAT_EQ( -1.0f, x);
    EXPECT_FLOAT_EQ( -7.0f, y);
    EXPECT_TRUE(retval2 == true);
}

TEST(TestSuiteWorldStateFunctionsGetPotentialOpponentAttacker, PotentialOpponentAttacker_test3)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: Bot1 @ 1.0,-6.0, Ball @ 1.1,-6.0  => only 1 opponent which is already closest to ball

    // set ball location
    teamplay::ballStore::getBall().setPosition(Point3D(1.1, -6.0, 0.0));

    // set other bot location(s) and stuff
    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(1.0, -6.0, 0.0)));

    // Execution
    float x,y;
    bool retval = cWorldStateFunctions::getInstance().getPotentialOpponentAttacker(x,y);
    std::map<std::string, std::string> params;
    bool retval2 = isPotentialOppAttackerPresent(params);

    // Verification
    EXPECT_TRUE(retval == false);
    EXPECT_TRUE(retval2 == false);
}


TEST(areTurnsToGoalBlockedByOpponent, Test1)
{
    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(-4.0, 2.0, M_PI), Velocity2D()));

    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(-4.0, 0.4, 0.0)));
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(-2.5, 3.5, 0.0)));
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(-1.0, 7.0, 0.0)));

    std::map<std::string, std::string> params;
    EXPECT_FALSE(isShortTurnToGoalBlockedByOpponent(params));
    EXPECT_FALSE(isLongTurnToGoalBlockedByOpponent(params));
}

TEST(areTurnsToGoalBlockedByOpponent, Test2)
{
    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(-4.0, 2.0, M_PI), Velocity2D()));

    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(-4.0, 0.4, 0.0)));
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(-4.0, 2.9, 0.0)));
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(-1.0, 7.0, 0.0)));

    std::map<std::string, std::string> params;
    EXPECT_TRUE(isShortTurnToGoalBlockedByOpponent(params));
    EXPECT_FALSE(isLongTurnToGoalBlockedByOpponent(params));
}

TEST(areTurnsToGoalBlockedByOpponent, Test3)
{
    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(-4.0, 2.0, 0.0), Velocity2D()));

    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(-4.0, 0.4, 0.0)));
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(-4.0, 2.9, 0.0)));
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(-1.0, 7.0, 0.0)));

    std::map<std::string, std::string> params;
    EXPECT_FALSE(isShortTurnToGoalBlockedByOpponent(params));
    EXPECT_TRUE(isLongTurnToGoalBlockedByOpponent(params));
}

TEST(areTurnsToGoalBlockedByOpponent, Test4)
{
    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(-4.0, 2.0, M_PI), Velocity2D()));

    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(-4.0, 1.1, 0.0)));
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(-4.0, 2.9, 0.0)));
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(-1.0, 7.0, 0.0)));

    std::map<std::string, std::string> params;
    EXPECT_TRUE(isShortTurnToGoalBlockedByOpponent(params));
    EXPECT_TRUE(isLongTurnToGoalBlockedByOpponent(params));
}

TEST(areTurnsToGoalBlockedByOpponent, Test5)
{
    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(0.0, 5.0, (1.4 * M_PI)), Velocity2D()));

    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(-0.5, 7.0, 0.0)));
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(0.5, 7.0, 0.0)));

    std::map<std::string, std::string> params;
    EXPECT_FALSE(isShortTurnToGoalBlockedByOpponent(params));
    EXPECT_FALSE(isLongTurnToGoalBlockedByOpponent(params));
}

TEST(areTurnsToGoalBlockedByOpponent, Test6)
{
    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(0.0, 5.0, (1.4 * M_PI)), Velocity2D()));

    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(-0.5, 5.5, 0.0)));
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(0.5, 7.0, 0.0)));

    std::map<std::string, std::string> params;
    EXPECT_TRUE(isShortTurnToGoalBlockedByOpponent(params));
    EXPECT_FALSE(isLongTurnToGoalBlockedByOpponent(params));
}

TEST(areTurnsToGoalBlockedByOpponent, Test7)
{
    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(0.0, 5.0, (1.4 * M_PI)), Velocity2D()));

    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(-0.5, 7.0, 0.0)));
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(0.5, 5.5, 0.0)));

    std::map<std::string, std::string> params;
    EXPECT_FALSE(isShortTurnToGoalBlockedByOpponent(params));
    EXPECT_TRUE(isLongTurnToGoalBlockedByOpponent(params));
}

TEST(areTurnsToGoalBlockedByOpponent, Test8)
{
    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(3.0, 7.0, (2.0 * M_PI)), Velocity2D()));

    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(2.5, 7.5, 0.0)));

    std::map<std::string, std::string> params;
    EXPECT_TRUE(isShortTurnToGoalBlockedByOpponent(params));
    EXPECT_FALSE(isLongTurnToGoalBlockedByOpponent(params));
}

TEST(areTurnsToGoalBlockedByOpponent, Test9)
{
    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(3.0, 7.0, (1.5 * M_PI)), Velocity2D()));

    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(2.5, 7.0, 0.0)));

    std::map<std::string, std::string> params;
    EXPECT_TRUE(isShortTurnToGoalBlockedByOpponent(params));
    EXPECT_FALSE(isLongTurnToGoalBlockedByOpponent(params));
}

// ======================
// getObstructingObstaclesInPath
// ======================

TEST(TestSuiteWorldStateFunctionsgetObstructingObstaclesInPath, TestObstructing1)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: OWN @0,0,
    // Bot1 @0,6, Bot 2 @0,4
    // Target @0,5
    // Radius ball = 0.125

    double radiusBall = 0.125;
    std::vector<robotLocation> obstacles;

    ConfigTeamplay config;
    config.shooting.shootPathWidth = 0.25;
    teamplay::configurationStore::getConfiguration().update(config);

    // set other bot location(s) and stuff
    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(0.0, 6.0, 0.0)));
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(0.0, 4.0, 0.0)));

    // Execution
    Point2D targetPos = Point2D(0.0, 5.0);
    Point2D robotPos = Point2D(0.0, 0.0);
    cWorldStateFunctions::getInstance().getObstructingObstaclesInPath(robotPos, targetPos, radiusBall, obstacles);

    // Verification
    EXPECT_TRUE(obstacles.size() == 1);
    EXPECT_FLOAT_EQ(obstacles.at(0).position.getX(), 0.0);
    EXPECT_FLOAT_EQ(obstacles.at(0).position.getY(), 4.0);

}

TEST(TestSuiteWorldStateFunctionsgetObstructingObstaclesInPath, TestObstructing2)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: OWN @0,0,
    // Bot1 @0,6, Bot 2 @0,5
    // Target @0,5
    // Radius ball = 0.125

    double radiusBall = 0.125;
    std::vector<robotLocation> obstacles;

    ConfigTeamplay config;
    config.shooting.shootPathWidth = 0.25;
    teamplay::configurationStore::getConfiguration().update(config);

    // set other bot location(s) and stuff
    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(0.0, 6.0, 0.0)));
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(0.0, 5.0, 0.0)));

    // Execution
    Point2D targetPos = Point2D(0.0, 5.0);
    Point2D robotPos = Point2D(0.0, 0.0);
    cWorldStateFunctions::getInstance().getObstructingObstaclesInPath(robotPos, targetPos, radiusBall, obstacles);

    // Verification
    EXPECT_TRUE(obstacles.size() == 1);
    EXPECT_FLOAT_EQ(obstacles.at(0).position.getX(), 0.0);
    EXPECT_FLOAT_EQ(obstacles.at(0).position.getY(), 5.0);
}

TEST(TestSuiteWorldStateFunctionsgetObstructingObstaclesInPath, TestObstructing3)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: OWN @0,0,
    // Bot1 @0,6, Bot 2 @0,4
    // Target @0,7
    // Radius ball = 0.125

    double radiusBall = 0.125;
    std::vector<robotLocation> obstacles;

    ConfigTeamplay config;
    config.shooting.shootPathWidth = 0.25;
    teamplay::configurationStore::getConfiguration().update(config);

    // set other bot location(s) and stuff
    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(0.0, 6.0, 0.0)));
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(0.0, 4.0, 0.0)));

    // Execution
    Point2D targetPos = Point2D(0.0, 7.0);
    Point2D robotPos = Point2D(0.0, 0.0);
    cWorldStateFunctions::getInstance().getObstructingObstaclesInPath(robotPos, targetPos, radiusBall, obstacles);

    // Verification
    EXPECT_TRUE(obstacles.size() == 2);
    EXPECT_FLOAT_EQ(obstacles.at(0).position.getX(), 0.0);
    EXPECT_FLOAT_EQ(obstacles.at(0).position.getY(), 4.0);
    EXPECT_FLOAT_EQ(obstacles.at(1).position.getX(), 0.0);
    EXPECT_FLOAT_EQ(obstacles.at(1).position.getY(), 6.0);
}

TEST(TestSuiteWorldStateFunctionsgetObstructingObstaclesInPath, ReceivingTeammember1)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: OWN @0,0,
    // Bot1 @0,6
    // Member2 @0,5
    // Target @0,5
    // Radius ball = 0.125

    double radiusBall = 0.125;
    std::vector<robotLocation> obstacles;

    ConfigTeamplay config;
    config.shooting.shootPathWidth = 0.25;
    teamplay::configurationStore::getConfiguration().update(config);

    // Clean the robot store and define one teammate
    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addTeammate(teamplay::robot(2, treeEnum::DEFENDER_MAIN, Position2D(0.0, 5.0, 0.0), Velocity2D()));

    // set other bot location(s) and stuff
    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(0.0, 6.0, 0.0)));

    // Execution
    Point2D targetPos = Point2D(0.0, 5.0);
    Point2D robotPos = Point2D(0.0, 0.0);
    cWorldStateFunctions::getInstance().getObstructingObstaclesInPath(robotPos, targetPos, radiusBall, obstacles);

    // Verification
    EXPECT_TRUE(obstacles.size() == 0);
}

TEST(TestSuiteWorldStateFunctionsisPassToClosestTeammemberBlocked, Teammember1)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: OWN @0,0,
    // Member2 @0,5
    // Bot1 @0,2
    // Radius ball = 0.125

    std::vector<robotLocation> obstacles;

    ConfigTeamplay config;
    config.shooting.shootPathWidth = 0.25;
    teamplay::configurationStore::getConfiguration().update(config);

    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(0.0, 0.0, 0.0), Velocity2D()));
    teamplay::robotStore::getInstance().addTeammate(teamplay::robot(2, treeEnum::DEFENDER_MAIN, Position2D(0.0, 5.0, 0.0), Velocity2D()));

    // set other bot location(s) and stuff
    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(0.0, 2.0, 0.0)));

    // Execution
    std::map<std::string, std::string> params;
    EXPECT_TRUE(isPassToClosestTeammemberBlocked(params));
}

TEST(TestSuiteWorldStateFunctionsisPassToClosestTeammemberBlocked, Teammember2)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: OWN @0,0,
    // Member2 @0,5
    // Bot1 @0,7
    // Radius ball = 0.125

    std::vector<robotLocation> obstacles;

    ConfigTeamplay config;
    config.shooting.shootPathWidth = 0.25;
    teamplay::configurationStore::getConfiguration().update(config);

    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(0.0, 0.0, 0.0), Velocity2D()));
    teamplay::robotStore::getInstance().addTeammate(teamplay::robot(2, treeEnum::DEFENDER_MAIN, Position2D(0.0, 5.0, 0.0), Velocity2D()));

    // set other bot location(s) and stuff
    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(0.0, 7.0, 0.0)));

    // Execution
    std::map<std::string, std::string> params;
    EXPECT_FALSE(isPassToClosestTeammemberBlocked(params));
}

TEST(TestSuiteWorldStateFunctionsisPassToFurthestDefenderBlocked, test1)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: OWN @0,2
    // Member1 (DEFENDER_ASSIST) @0,0
    // Member2 (DEFENDER_MAIN) @0,-2
    // Add obstacle @0,1
    // Radius ball = 0.125

    std::vector<robotLocation> obstacles;

    ConfigTeamplay config;
    config.shooting.shootPathWidth = 0.25;
    teamplay::configurationStore::getConfiguration().update(config);

    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(0.0, 2.0, 0.0), Velocity2D()));
    teamplay::robotStore::getInstance().addTeammate(teamplay::robot(2, treeEnum::DEFENDER_MAIN, Position2D(4.0, -2.0, 0.0), Velocity2D()));
    teamplay::robotStore::getInstance().addTeammate(teamplay::robot(3, treeEnum::DEFENDER_ASSIST, Position2D(0.0, 0.0, 0.0), Velocity2D()));

    // set other bot location(s) and stuff
    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(0.0, 1.0, 0.0)));

    // Execution
    std::map<std::string, std::string> params;
    EXPECT_TRUE(isPassToFurthestDefenderBlocked(params));
}

TEST(TestSuiteWorldStateFunctionsisPassToFurthestDefenderBlocked, test2)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: OWN @0,2
    // Member1 (DEFENDER_ASSIST) @0,0
    // Member2 (DEFENDER_MAIN) @0,-2
    // Add obstacle @4,1
    // Radius ball = 0.125

    std::vector<robotLocation> obstacles;

    ConfigTeamplay config;
    config.shooting.shootPathWidth = 0.25;
    teamplay::configurationStore::getConfiguration().update(config);

    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(4.0, 2.0, 0.0), Velocity2D()));
    teamplay::robotStore::getInstance().addTeammate(teamplay::robot(2, treeEnum::DEFENDER_MAIN, Position2D(4.0, -2.0, 0.0), Velocity2D()));
    teamplay::robotStore::getInstance().addTeammate(teamplay::robot(3, treeEnum::DEFENDER_ASSIST, Position2D(0.0, 0.0, 0.0), Velocity2D()));

    // set other bot location(s) and stuff
    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(4.0, 0.0, 0.0)));

    // Execution
    std::map<std::string, std::string> params;
    EXPECT_FALSE(isPassToFurthestDefenderBlocked(params));
}

TEST(TestSuiteWorldStateFunctionsisLobShotOnGoalBlocked, Opponent1)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: OWN @0,0,
    // Bot1 @0,1
    // Radius ball = 0.125

    std::vector<robotLocation> obstacles;

    ConfigTeamplay config;
    config.shooting.shootPathWidth = 0.25;
    teamplay::configurationStore::getConfiguration().update(config);

    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(0.0, 0.0, 0.0), Velocity2D()));

    // Set teammember on Target
    std::vector<robotNumber> activeRobots;
    robotLocations teammembers;

    // set other bot location(s) and stuff
    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(0.0, 1.0, 0.0)));

    // Execution
    std::map<std::string, std::string> params;
    EXPECT_FALSE(isLobShotOnGoalBlocked(params));
}

TEST(TestSuiteWorldStateFunctionsisPathToBallBlocked, PathBlocked_test1)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: OWN @0,0,
    // Bal @0,6
    // Bot1 @0,4 --> robot is obstructing path ball<>robot
    // Radius ball = 0.125

    std::vector<robotLocation> obstacles;

    ConfigTeamplay config;
    config.shooting.shootPathWidth = 0.25;
    teamplay::configurationStore::getConfiguration().update(config);

    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(0.0, 0.0, 0.0), Velocity2D()));

    // set ball location
    teamplay::ballStore::getBall().setPosition(Point3D(0.0, 6.0, 0.0));

    // set other bot location(s) and stuff
    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(0.0, 4.0, 0.0)));

    // Execution
    std::map<std::string, std::string> params;
    EXPECT_TRUE(isPathToBallBlocked(params));
}

TEST(TestSuiteWorldStateFunctionsisPathToBallBlocked, PathBlocked_test2)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: OWN @0,0,
    // Bal @0,6
    // Bot1 @3,3 --> robot is NOT obstructing path ball<>robot
    // Radius ball = 0.125

    std::vector<robotLocation> obstacles;

    ConfigTeamplay config;
    config.shooting.shootPathWidth = 0.25;
    teamplay::configurationStore::getConfiguration().update(config);

    teamplay::robotStore::getInstance().clear();

    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(0.0, 0.0, 0.0), Velocity2D()));

    // set ball location
    teamplay::ballStore::getBall().setPosition(Point3D(0.0, 6.0, 0.0));

    // set other bot location(s) and stuff
    teamplay::obstacleStore::getInstance().clear();
    teamplay::obstacleStore::getInstance().addObstacle(obstacle(Position2D(3.0, 3.0, 0.0)));

    // Execution
    std::map<std::string, std::string> params;
    EXPECT_FALSE(isPathToBallBlocked(params));
}

TEST(TestSuiteWorldStateFunctionsDefendingStrategyOnTest, defendingStrategyTest)
{
    // Setup; set setDefendingStrategy to true

    ConfigTeamplay config;
    config.strategy.defendingStrategy = true;
    teamplay::configurationStore::getConfiguration().update(config);
    std::map<std::string, std::string> params;
    bool retval = defendingStrategyOn(params);

    // Verification
    EXPECT_TRUE(retval == true);
}

TEST(TestSuiteWorldStateFunctionsDefendingStrategyOnTest, defendingStrategyTest2)
{
    // Setup; set setDefendingStrategy to false

    ConfigTeamplay config;
    config.strategy.defendingStrategy = false;
    teamplay::configurationStore::getConfiguration().update(config);
    std::map<std::string, std::string> params;
    bool retval = defendingStrategyOn(params);

    // Verification
    EXPECT_TRUE(retval == false);
}

TEST(TestSuiteWorldStateFunctionsisOpponentHalfReachable, OpponentHalfReachable_test1)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: OWN @0,-0.1,
    // Ball same as own.picked up at (0, 0.1)

    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(0.0, -0.1, 0.0), Velocity2D()));

    // set ball location
    teamplay::ballStore::getBall().setPosition(Point3D(0.0, 0.1, 0.0));
    teamplay::ballStore::getBall().setPositionClaimed(Point3D(0.0, 0.1, 0.0));

    // Execution
    std::map<std::string, std::string> params;
    EXPECT_TRUE(isOpponentHalfReachable(params));
}

TEST(TestSuiteWorldStateFunctionsisOpponentHalfReachable, OpponentHalfReachable_test2)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: OWN @0,-1.5,
    // Ball same as own.picked up at (0, -1.5)

    teamplay::robotStore::getInstance().clear();
    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, Position2D(0.0, -1.5, 0.0), Velocity2D()));

    // set ball location
    teamplay::ballStore::getBall().setPosition(Point3D(0.0, 1.5, 0.0));
    teamplay::ballStore::getBall().setPositionClaimed(Point3D(0.0, 1.5, 0.0));

    // Execution
    std::map<std::string, std::string> params;
    EXPECT_TRUE(isOpponentHalfReachable(params));
}

TEST(TestSuiteWorldStateFunctionsClosestAttackerToOppGoal, closestAttToOppGoal_test1)
{
    // Setup
    // Update WorldModel administration
    // TESTCASE: OWN @0,0,
    // Member1 @0,5
    // Member2 @0,7

    teamplay::robotStore::getInstance().clear();

    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::R_GOALKEEPER, Position2D(0.0, 0.0, 0.0), Velocity2D()));
    teamplay::robotStore::getInstance().addTeammate(teamplay::robot(2, treeEnum::ATTACKER_MAIN, Position2D(0.0, 5.0, 0.0), Velocity2D()));
    teamplay::robotStore::getInstance().addTeammate(teamplay::robot(3, treeEnum::ATTACKER_ASSIST, Position2D(0.0, 7.0, 0.0), Velocity2D()));

    // Execution
    auto result = cWorldStateFunctions::getInstance().getClosestAttackerToOpponentGoal();

    // Verification
    ASSERT_TRUE((bool)result);
    EXPECT_EQ(0.0, result->getLocation().x) << "Wrong x position";
    EXPECT_EQ(7.0, result->getLocation().y) << "Wrong y position";
}


class GameStateTest : public Test { };

TEST_F(GameStateTest, CorrectlyStoresNeutralPlaying)
{
    gameState g(governingGameState::NEUTRAL_PLAYING);
    EXPECT_TRUE(g.isInMatch());
    EXPECT_FALSE(g.isSetPiece());
    EXPECT_FALSE(g.isOwnSetPiece());
}

TEST_F(GameStateTest, CorrectlyStoresAValidSetpiece)
{
    gameState g(governingGameState::SETPIECE_PREPARING, governingMatchState::IN_MATCH, setpieceOwner::OPPONENT, setpieceType::CORNER);
    EXPECT_TRUE(g.isInMatch());
    EXPECT_TRUE(g.isSetPiece());
    EXPECT_FALSE(g.isOwnSetPiece());
}

TEST_F(GameStateTest, DoesNotStoreASetpieceWithoutAttributes)
{
    EXPECT_ANY_THROW(gameState g(governingGameState::SETPIECE_PREPARING));
}

TEST_F(GameStateTest, DoesNotStoreNeutralPlayingWithSetpieceAttributes)
{
    EXPECT_ANY_THROW(gameState g(governingGameState::NEUTRAL_PLAYING, governingMatchState::IN_MATCH, setpieceOwner::OPPONENT, setpieceType::CORNER));
}

TEST_F(GameStateTest, EqualityTest)
{
    gameState g1(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::FREEKICK);
    gameState g2(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::FREEKICK);
    gameState g3(governingGameState::SETPIECE_EXECUTING, governingMatchState::IN_MATCH, setpieceOwner::OWN, setpieceType::GOALKICK);
    gameState g4(governingGameState::NEUTRAL_PLAYING);

    EXPECT_TRUE(g1 == g2);
    EXPECT_FALSE(g1 != g2);

    EXPECT_FALSE(g2 == g3);
    EXPECT_TRUE(g2 != g3);

    EXPECT_FALSE(g3 == g4);
    EXPECT_TRUE(g3 != g4);

    EXPECT_TRUE(g4 == g4);
    EXPECT_FALSE(g4 != g4);
}


class GameStateQueriesTest : public Test { };

TEST_F(GameStateQueriesTest, InvalidGameState)
{
    gameStateStore::getInstance().updateGameState(governingGameState::INVALID);
    EXPECT_EQ(treeEnum::INVALID, gameStateStore::getInstance().getGameState_treeEnum());

    // Do not test playstate, it doesn't matter when the state is invalid.
    std::map<std::string, std::string> params;
    EXPECT_FALSE(isSetPiece(params));
    EXPECT_FALSE(isOwnSetPiece(params));
    EXPECT_FALSE(isPrepareSetPiece(params));
    EXPECT_FALSE(isDroppedBallSetPiece(params));
    EXPECT_FALSE(isKickoffSetPiece(params));
    EXPECT_FALSE(isPenaltySetPiece(params));
}

TEST_F(GameStateQueriesTest, NeutralStopped)
{
    gameStateStore::getInstance().updateGameState(governingGameState::NEUTRAL_STOPPED);
    EXPECT_EQ(treeEnum::IN_MATCH_NEUTRAL_STOPPED_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    std::map<std::string, std::string> params;
    EXPECT_TRUE(isInMatch(params));
    EXPECT_FALSE(isSetPiece(params));
    EXPECT_FALSE(isOwnSetPiece(params));
    EXPECT_FALSE(isPrepareSetPiece(params));
    EXPECT_FALSE(isDroppedBallSetPiece(params));
    EXPECT_FALSE(isKickoffSetPiece(params));
    EXPECT_FALSE(isPenaltySetPiece(params));
}

TEST_F(GameStateQueriesTest, NeutralStopped_treeEnumVariant)
{
    gameStateStore::getInstance().updateGameState(treeEnum::IN_MATCH_NEUTRAL_STOPPED_NEUTRAL);
    EXPECT_EQ(treeEnum::IN_MATCH_NEUTRAL_STOPPED_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    std::map<std::string, std::string> params;
    EXPECT_TRUE(isInMatch(params));
    EXPECT_FALSE(isSetPiece(params));
    EXPECT_FALSE(isOwnSetPiece(params));
    EXPECT_FALSE(isPrepareSetPiece(params));
    EXPECT_FALSE(isDroppedBallSetPiece(params));
    EXPECT_FALSE(isKickoffSetPiece(params));
    EXPECT_FALSE(isPenaltySetPiece(params));
}

TEST_F(GameStateQueriesTest, NeutralPlaying)
{
    gameStateStore::getInstance().updateGameState(governingGameState::NEUTRAL_PLAYING);
    EXPECT_EQ(treeEnum::IN_MATCH_NEUTRAL_PLAYING_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    std::map<std::string, std::string> params;
    EXPECT_TRUE(isInMatch(params));
    EXPECT_FALSE(isSetPiece(params));
    EXPECT_FALSE(isOwnSetPiece(params));
    EXPECT_FALSE(isPrepareSetPiece(params));
    EXPECT_FALSE(isDroppedBallSetPiece(params));
    EXPECT_FALSE(isKickoffSetPiece(params));
    EXPECT_FALSE(isPenaltySetPiece(params));
}

TEST_F(GameStateQueriesTest, DroppedBallPrepare)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_PREPARING, setpieceOwner::NONE, setpieceType::DROPPED_BALL);
    EXPECT_EQ(treeEnum::IN_MATCH_DROPPED_BALL_PREPARE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    std::map<std::string, std::string> params;
    EXPECT_TRUE(isInMatch(params));
    EXPECT_TRUE(isSetPiece(params));

    /* A dropped ball is a setpiece for both teams. Design decision:
     * a dropped ball is neither an own setpiece, nor an opponent setpiece*/
    EXPECT_FALSE(isOwnSetPiece(params));
    EXPECT_TRUE(isPrepareSetPiece(params));
    EXPECT_TRUE(isDroppedBallSetPiece(params));
    EXPECT_FALSE(isKickoffSetPiece(params));
    EXPECT_FALSE(isPenaltySetPiece(params));
}

TEST_F(GameStateQueriesTest, DroppedBallPrepare_treeEnumVariant)
{
    gameStateStore::getInstance().updateGameState(treeEnum::IN_MATCH_DROPPED_BALL_PREPARE_NEUTRAL);
    EXPECT_EQ(treeEnum::IN_MATCH_DROPPED_BALL_PREPARE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    std::map<std::string, std::string> params;
    EXPECT_TRUE(isInMatch(params));
    EXPECT_TRUE(isSetPiece(params));

    /* A dropped ball is a setpiece for both teams. Design decision:
     * a dropped ball is neither an own setpiece, nor an opponent setpiece*/
    EXPECT_FALSE(isOwnSetPiece(params));
    EXPECT_TRUE(isPrepareSetPiece(params));
    EXPECT_TRUE(isDroppedBallSetPiece(params));
    EXPECT_FALSE(isKickoffSetPiece(params));
    EXPECT_FALSE(isPenaltySetPiece(params));
}

TEST_F(GameStateQueriesTest, DroppedBallExecute)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::NONE, setpieceType::DROPPED_BALL);
    EXPECT_EQ(treeEnum::IN_MATCH_DROPPED_BALL_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    std::map<std::string, std::string> params;
    EXPECT_TRUE(isInMatch(params));
    EXPECT_TRUE(isSetPiece(params));

    /* A dropped ball is a setpiece for both teams. Design decision:
     * a dropped ball is neither an own setpiece, nor an opponent setpiece*/
    EXPECT_FALSE(isOwnSetPiece(params));
    EXPECT_FALSE(isPrepareSetPiece(params));
    EXPECT_TRUE(isDroppedBallSetPiece(params));
    EXPECT_FALSE(isKickoffSetPiece(params));
    EXPECT_FALSE(isPenaltySetPiece(params));
}

TEST_F(GameStateQueriesTest, OwnKickoffPrepare)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_PREPARING, setpieceOwner::OWN, setpieceType::KICKOFF);
    EXPECT_EQ(treeEnum::IN_MATCH_OWN_KICKOFF_PREPARE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    std::map<std::string, std::string> params;
    EXPECT_TRUE(isInMatch(params));
    EXPECT_TRUE(isSetPiece(params));
    EXPECT_TRUE(isOwnSetPiece(params));
    EXPECT_TRUE(isPrepareSetPiece(params));
    EXPECT_FALSE(isDroppedBallSetPiece(params));
    EXPECT_TRUE(isKickoffSetPiece(params));
    EXPECT_FALSE(isPenaltySetPiece(params));
}

TEST_F(GameStateQueriesTest, OwnKickoffExecute)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OWN, setpieceType::KICKOFF);
    EXPECT_EQ(treeEnum::IN_MATCH_OWN_KICKOFF_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    std::map<std::string, std::string> params;
    EXPECT_TRUE(isInMatch(params));
    EXPECT_TRUE(isSetPiece(params));
    EXPECT_TRUE(isOwnSetPiece(params));
    EXPECT_FALSE(isPrepareSetPiece(params));
    EXPECT_FALSE(isDroppedBallSetPiece(params));
    EXPECT_TRUE(isKickoffSetPiece(params));
    EXPECT_FALSE(isPenaltySetPiece(params));
}

TEST_F(GameStateQueriesTest, OpponentKickoffPrepare)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_PREPARING, setpieceOwner::OPPONENT, setpieceType::KICKOFF);
    EXPECT_EQ(treeEnum::IN_MATCH_OPP_KICKOFF_PREPARE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    std::map<std::string, std::string> params;
    EXPECT_TRUE(isInMatch(params));
    EXPECT_TRUE(isSetPiece(params));
    EXPECT_FALSE(isOwnSetPiece(params));
    EXPECT_TRUE(isPrepareSetPiece(params));
    EXPECT_FALSE(isDroppedBallSetPiece(params));
    EXPECT_TRUE(isKickoffSetPiece(params));
    EXPECT_FALSE(isPenaltySetPiece(params));
}

TEST_F(GameStateQueriesTest, OpponentKickoffExecute)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OPPONENT, setpieceType::KICKOFF);
    EXPECT_EQ(treeEnum::IN_MATCH_OPP_KICKOFF_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    std::map<std::string, std::string> params;
    EXPECT_TRUE(isInMatch(params));
    EXPECT_TRUE(isSetPiece(params));
    EXPECT_FALSE(isOwnSetPiece(params));
    EXPECT_FALSE(isPrepareSetPiece(params));
    EXPECT_FALSE(isDroppedBallSetPiece(params));
    EXPECT_TRUE(isKickoffSetPiece(params));
    EXPECT_FALSE(isPenaltySetPiece(params));
}

TEST_F(GameStateQueriesTest, OwnFreekickExecute)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OWN, setpieceType::FREEKICK);
    EXPECT_EQ(treeEnum::IN_MATCH_OWN_FREEKICK_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    std::map<std::string, std::string> params;
    EXPECT_TRUE(isInMatch(params));
    EXPECT_TRUE(isSetPiece(params));
    EXPECT_TRUE(isOwnSetPiece(params));
    EXPECT_FALSE(isPrepareSetPiece(params));
    EXPECT_FALSE(isDroppedBallSetPiece(params));
    EXPECT_FALSE(isKickoffSetPiece(params));
    EXPECT_FALSE(isPenaltySetPiece(params));
}

TEST_F(GameStateQueriesTest, OwnGoalkickExecute)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OWN, setpieceType::GOALKICK);
    EXPECT_EQ(treeEnum::IN_MATCH_OWN_GOALKICK_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    std::map<std::string, std::string> params;
    EXPECT_TRUE(isInMatch(params));
    EXPECT_TRUE(isSetPiece(params));
    EXPECT_TRUE(isOwnSetPiece(params));
    EXPECT_FALSE(isPrepareSetPiece(params));
    EXPECT_FALSE(isDroppedBallSetPiece(params));
    EXPECT_FALSE(isKickoffSetPiece(params));
    EXPECT_FALSE(isPenaltySetPiece(params));
}

TEST_F(GameStateQueriesTest, OwnThrowinExecute)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OWN, setpieceType::THROWIN);
    EXPECT_EQ(treeEnum::IN_MATCH_OWN_THROWIN_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    std::map<std::string, std::string> params;
    EXPECT_TRUE(isInMatch(params));
    EXPECT_TRUE(isSetPiece(params));
    EXPECT_TRUE(isOwnSetPiece(params));
    EXPECT_FALSE(isPrepareSetPiece(params));
    EXPECT_FALSE(isDroppedBallSetPiece(params));
    EXPECT_FALSE(isKickoffSetPiece(params));
    EXPECT_FALSE(isPenaltySetPiece(params));
}

TEST_F(GameStateQueriesTest, OwnCornerExecute)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OWN, setpieceType::CORNER);
    EXPECT_EQ(treeEnum::IN_MATCH_OWN_CORNER_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    std::map<std::string, std::string> params;
    EXPECT_TRUE(isInMatch(params));
    EXPECT_TRUE(isSetPiece(params));
    EXPECT_TRUE(isOwnSetPiece(params));
    EXPECT_FALSE(isPrepareSetPiece(params));
    EXPECT_FALSE(isDroppedBallSetPiece(params));
    EXPECT_FALSE(isKickoffSetPiece(params));
    EXPECT_FALSE(isPenaltySetPiece(params));
}

TEST_F(GameStateQueriesTest, OwnPenaltyExecute)
{
    gameStateStore::getInstance().updateGameState(governingGameState::SETPIECE_EXECUTING, setpieceOwner::OWN, setpieceType::PENALTY);
    EXPECT_EQ(treeEnum::IN_MATCH_OWN_PENALTY_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    std::map<std::string, std::string> params;
    EXPECT_TRUE(isInMatch(params));
    EXPECT_TRUE(isSetPiece(params));
    EXPECT_TRUE(isOwnSetPiece(params));
    EXPECT_FALSE(isPrepareSetPiece(params));
    EXPECT_FALSE(isDroppedBallSetPiece(params));
    EXPECT_FALSE(isKickoffSetPiece(params));
    EXPECT_TRUE(isPenaltySetPiece(params));
}

TEST_F(GameStateQueriesTest, OwnPenaltyExecute_treeEnumVariant)
{
    gameStateStore::getInstance().updateGameState(treeEnum::IN_MATCH_OWN_PENALTY_EXECUTE_NEUTRAL);
    EXPECT_EQ(treeEnum::IN_MATCH_OWN_PENALTY_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    std::map<std::string, std::string> params;
    EXPECT_TRUE(isInMatch(params));
    EXPECT_TRUE(isSetPiece(params));
    EXPECT_TRUE(isOwnSetPiece(params));
    EXPECT_FALSE(isPrepareSetPiece(params));
    EXPECT_FALSE(isDroppedBallSetPiece(params));
    EXPECT_FALSE(isKickoffSetPiece(params));
    EXPECT_TRUE(isPenaltySetPiece(params));
}

TEST_F(GameStateQueriesTest, OutOfMatchNeutralStopped_treeEnumVariant)
{
    gameStateStore::getInstance().updateGameState(treeEnum::OUT_OF_MATCH_NEUTRAL_STOPPED_NEUTRAL);
    EXPECT_EQ(treeEnum::OUT_OF_MATCH_NEUTRAL_STOPPED_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    std::map<std::string, std::string> params;
    EXPECT_FALSE(isInMatch(params));
    EXPECT_FALSE(isSetPiece(params));
    EXPECT_FALSE(isOwnSetPiece(params));
    EXPECT_FALSE(isPrepareSetPiece(params));
    EXPECT_FALSE(isDroppedBallSetPiece(params));
    EXPECT_FALSE(isKickoffSetPiece(params));
    EXPECT_FALSE(isPenaltySetPiece(params));
}

TEST_F(GameStateQueriesTest, OutOfMatchOwnPenaltyPrepare_treeEnumVariant)
{
    gameStateStore::getInstance().updateGameState(treeEnum::OUT_OF_MATCH_OWN_PENALTY_PREPARE_NEUTRAL);
    EXPECT_EQ(treeEnum::OUT_OF_MATCH_OWN_PENALTY_PREPARE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    std::map<std::string, std::string> params;
    EXPECT_FALSE(isInMatch(params));
    EXPECT_TRUE(isSetPiece(params));
    EXPECT_TRUE(isOwnSetPiece(params));
    EXPECT_TRUE(isPrepareSetPiece(params));
    EXPECT_FALSE(isDroppedBallSetPiece(params));
    EXPECT_FALSE(isKickoffSetPiece(params));
    EXPECT_TRUE(isPenaltySetPiece(params));
}

TEST_F(GameStateQueriesTest, OutOfMatchOwnPenaltyExecute_treeEnumVariant)
{
    gameStateStore::getInstance().updateGameState(treeEnum::OUT_OF_MATCH_OWN_PENALTY_EXECUTE_NEUTRAL);
    EXPECT_EQ(treeEnum::OUT_OF_MATCH_OWN_PENALTY_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    std::map<std::string, std::string> params;
    EXPECT_FALSE(isInMatch(params));
    EXPECT_TRUE(isSetPiece(params));
    EXPECT_TRUE(isOwnSetPiece(params));
    EXPECT_FALSE(isPrepareSetPiece(params));
    EXPECT_FALSE(isDroppedBallSetPiece(params));
    EXPECT_FALSE(isKickoffSetPiece(params));
    EXPECT_TRUE(isPenaltySetPiece(params));
}

TEST_F(GameStateQueriesTest, OutOfMatchOpponentPenaltyPrepare_treeEnumVariant)
{
    gameStateStore::getInstance().updateGameState(treeEnum::OUT_OF_MATCH_OPP_PENALTY_PREPARE_NEUTRAL);
    EXPECT_EQ(treeEnum::OUT_OF_MATCH_OPP_PENALTY_PREPARE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    std::map<std::string, std::string> params;
    EXPECT_FALSE(isInMatch(params));
    EXPECT_TRUE(isSetPiece(params));
    EXPECT_FALSE(isOwnSetPiece(params));
    EXPECT_TRUE(isPrepareSetPiece(params));
    EXPECT_FALSE(isDroppedBallSetPiece(params));
    EXPECT_FALSE(isKickoffSetPiece(params));
    EXPECT_TRUE(isPenaltySetPiece(params));
}

TEST_F(GameStateQueriesTest, OutOfMatchOpponentPenaltyExecute_treeEnumVariant)
{
    gameStateStore::getInstance().updateGameState(treeEnum::OUT_OF_MATCH_OPP_PENALTY_EXECUTE_NEUTRAL);
    EXPECT_EQ(treeEnum::OUT_OF_MATCH_OPP_PENALTY_EXECUTE_NEUTRAL, gameStateStore::getInstance().getGameState_treeEnum());

    std::map<std::string, std::string> params;
    EXPECT_FALSE(isInMatch(params));
    EXPECT_TRUE(isSetPiece(params));
    EXPECT_FALSE(isOwnSetPiece(params));
    EXPECT_FALSE(isPrepareSetPiece(params));
    EXPECT_FALSE(isDroppedBallSetPiece(params));
    EXPECT_FALSE(isKickoffSetPiece(params));
    EXPECT_TRUE(isPenaltySetPiece(params));
}


class BallQueriesTest : public Test { };

TEST_F(BallQueriesTest, ballAtSide1)
{
    ballStore::getBall().setPosition(Point3D(0.0, 1.0, 0.0)); //opponent half
    ballStore::getBall().setPositionUnknown();
    std::map<std::string, std::string> params;
    params["ball_own_opp"] = "OWN";
    params["ball_left_right"] = emptyValue;
    EXPECT_FALSE(isBallAtSide(params));
    params["ball_own_opp"] = "OPP";
    params["ball_left_right"] = emptyValue;
    EXPECT_TRUE(isBallAtSide(params));
}

TEST_F(BallQueriesTest, ballAtSide2)
{
    ballStore::getBall().setPosition(Point3D(1.0, 0.0, 0.0)); //right half
    ballStore::getBall().setPositionUnknown();
    std::map<std::string, std::string> params;
    params["ball_own_opp"] = emptyValue;
    params["ball_left_right"] = "LEFT";
    EXPECT_FALSE(isBallAtSide(params));
    params["ball_own_opp"] = emptyValue;
    params["ball_left_right"] = "RIGHT";
    EXPECT_TRUE(isBallAtSide(params));
}

TEST_F(BallQueriesTest, ballAtSide3)
{
    ballStore::getBall().setPosition(Point3D(-1.0, -1.0, 0.0)); //own half, left side
    ballStore::getBall().setPositionUnknown();
    std::map<std::string, std::string> params;
    params["ball_own_opp"] = "OPP";
    params["ball_left_right"] = "RIGHT";
    EXPECT_FALSE(isBallAtSide(params));
    params["ball_own_opp"] = "OPP";
    params["ball_left_right"] = "LEFT";
    EXPECT_FALSE(isBallAtSide(params));
    params["ball_own_opp"] = "OWN";
    params["ball_left_right"] = "RIGHT";
    EXPECT_FALSE(isBallAtSide(params));
    params["ball_own_opp"] = "OWN";
    params["ball_left_right"] = "LEFT";
    EXPECT_TRUE(isBallAtSide(params));
}

class TeamTest : public Test
{
public:
    TeamTest()
{
        robotStore::getInstance().clear();
        robotStore::getInstance().addOwnRobot(robot(1, treeEnum::R_GOALKEEPER, Position2D(), Velocity2D()));
        robotStore::getInstance().addTeammate(robot(2, treeEnum::DEFENDER_MAIN, Position2D(), Velocity2D()));
        robotStore::getInstance().addTeammate(robot(3, treeEnum::ATTACKER_MAIN, Position2D(), Velocity2D()));
        robotStore::getInstance().addTeammate(robot(4, treeEnum::DEFENDER_ASSIST, Position2D(), Velocity2D()));
        robotStore::getInstance().addTeammate(robot(5, treeEnum::ATTACKER_ASSIST, Position2D(), Velocity2D()));
}
};

TEST_F(TeamTest, ownRobotIsLowestActiveRobot)
{
    std::map<std::string, std::string> params;
    EXPECT_TRUE(isLowestActiveRobotID(params));
    EXPECT_FALSE(isSecondHighestActiveRobotID(params));
    EXPECT_FALSE(isThirdHighestActiveRobotID(params));
    EXPECT_FALSE(isHighestActiveRobotID(params));
}

TEST_F(TeamTest, ownRobotIsHighestActiveRobot)
{
    robotStore::getInstance().exchangeOwnRobotWith(5);

    std::map<std::string, std::string> params;
    EXPECT_FALSE(isLowestActiveRobotID(params));
    EXPECT_FALSE(isSecondHighestActiveRobotID(params));
    EXPECT_FALSE(isThirdHighestActiveRobotID(params));
    EXPECT_TRUE(isHighestActiveRobotID(params));

    robotStore::getInstance().undoExchange();
}

TEST_F(TeamTest, ownRobotIsSecondHighestActiveRobot)
{
    robotStore::getInstance().exchangeOwnRobotWith(4);

    std::map<std::string, std::string> params;
    EXPECT_FALSE(isLowestActiveRobotID(params));
    EXPECT_TRUE(isSecondHighestActiveRobotID(params));
    EXPECT_FALSE(isThirdHighestActiveRobotID(params));
    EXPECT_FALSE(isHighestActiveRobotID(params));

    robotStore::getInstance().undoExchange();
}

TEST_F(TeamTest, ownRobotIsThirdHighestActiveRobot)
{
    robotStore::getInstance().exchangeOwnRobotWith(3);

    std::map<std::string, std::string> params;
    EXPECT_FALSE(isLowestActiveRobotID(params));
    EXPECT_FALSE(isSecondHighestActiveRobotID(params));
    EXPECT_TRUE(isThirdHighestActiveRobotID(params));
    EXPECT_FALSE(isHighestActiveRobotID(params));

    robotStore::getInstance().undoExchange();
}

TEST_F(TeamTest, ownRobotIsOnlyActiveRobot)
{
    std::map<std::string, std::string> params;
    EXPECT_FALSE(isOnlyActiveRobotID(params));

    robotStore::getInstance().clear();
    robotStore::getInstance().addOwnRobot(robot(1, treeEnum::R_GOALKEEPER, Position2D(), Velocity2D()));
    EXPECT_TRUE(isOnlyActiveRobotID(params));
}


class PositionsTest : public Test
{
public:
    PositionsTest()
{
        robots.push_back(robot(1, treeEnum::R_GOALKEEPER, Position2D(-3.0, -4.1, 0.0), Velocity2D()));
        robots.push_back(robot(2, treeEnum::DEFENDER_MAIN, Position2D(3.0, -4.2, 0.0), Velocity2D()));
        robots.push_back(robot(3, treeEnum::ATTACKER_MAIN, Position2D(0.0, 0.5, 0.0), Velocity2D()));
        robots.push_back(robot(4, treeEnum::DEFENDER_ASSIST, Position2D(-2.5, 1.5, 0.0), Velocity2D()));
        robots.push_back(robot(5, treeEnum::ATTACKER_ASSIST, Position2D(2.0, 1.0, 0.0), Velocity2D()));
}

    teamplay::robots robots;
};

TEST_F(PositionsTest, getRobotClosestToPoint)
{
    EXPECT_EQ(1, getRobotClosestToPoint(robots, Point2D(-3.0, -4.1)).getNumber());
    EXPECT_EQ(1, getRobotClosestToPoint(robots, Point2D(-2.0, -3.0)).getNumber());
    EXPECT_EQ(1, getRobotClosestToPoint(robots, Point2D(-3.0, -10.0)).getNumber());
    EXPECT_EQ(2, getRobotClosestToPoint(robots, Point2D(2.0, -2.0)).getNumber());
    EXPECT_EQ(3, getRobotClosestToPoint(robots, Point2D(0.0, 0.0)).getNumber());
    EXPECT_EQ(4, getRobotClosestToPoint(robots, Point2D(-2.0, 1.0)).getNumber());
    EXPECT_EQ(5, getRobotClosestToPoint(robots, Point2D(1.5, 0.5)).getNumber());
}

TEST_F(PositionsTest, throwsWhenNoRobotsGiven)
{
    robots.clear();
    EXPECT_ANY_THROW(getRobotClosestToPoint(robots, Point2D(0.0, 0.0)));
}

TEST_F(PositionsTest, ownRobotClosestToPOIball)
{
    //initalize extra parameters;
    robotStore::getInstance().clear();
    robotStore::getInstance().addOwnRobot(teamplay::robot(2, treeEnum::DEFENDER_MAIN, Position2D(0.0, -2.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(1, treeEnum::R_GOALKEEPER, Position2D(0.0, -8.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(3, treeEnum::ATTACKER_MAIN, Position2D(0.0, 5.0, 0.0), Velocity2D()));
    ballStore::getBall().setPosition(Point3D(0.0, 0.0, 0.0)); //ball at center

    std::map<std::string, std::string> params;
    params["POI"] = "ball";
    EXPECT_TRUE(isOwnRobotClosestToPOI(params));
}

TEST_F(PositionsTest, ownRobotNotClosestToPOIball)
{
    //initalize extra parameters;
    robotStore::getInstance().clear();
    robotStore::getInstance().addOwnRobot(teamplay::robot(2, treeEnum::DEFENDER_MAIN, Position2D(0.0, -7.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(1, treeEnum::R_GOALKEEPER, Position2D(0.0, -8.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(3, treeEnum::ATTACKER_MAIN, Position2D(0.0, 5.0, 0.0), Velocity2D()));
    ballStore::getBall().setPosition(Point3D(0.0, 0.0, 0.0)); //ball at center
    std::map<std::string, std::string> params;
    params["POI"] = "ball";

    EXPECT_FALSE(isOwnRobotClosestToPOI(params));
}

TEST_F(PositionsTest, ownRobotSecondClosestToPOIball)
{
    //initalize extra parameters;
    robotStore::getInstance().clear();
    robotStore::getInstance().addOwnRobot(teamplay::robot(2, treeEnum::DEFENDER_MAIN, Position2D(0.0, -7.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(1, treeEnum::R_GOALKEEPER, Position2D(0.0, -8.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(3, treeEnum::ATTACKER_MAIN, Position2D(0.0, 5.0, 0.0), Velocity2D()));
    ballStore::getBall().setPosition(Point3D(0.0, 0.0, 0.0)); //ball at center
    std::map<std::string, std::string> params;
    params["POI"] = "ball";

    EXPECT_TRUE(isOwnRobotSecondClosestToPOI(params));
}

TEST_F(PositionsTest, ownRobotNotSecondClosestToPOIball)
{
    //initalize extra parameters;
    robotStore::getInstance().clear();
    robotStore::getInstance().addOwnRobot(teamplay::robot(2, treeEnum::DEFENDER_MAIN, Position2D(0.0, -1.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(1, treeEnum::R_GOALKEEPER, Position2D(0.0, -8.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(3, treeEnum::ATTACKER_MAIN, Position2D(0.0, 5.0, 0.0), Velocity2D()));
    ballStore::getBall().setPosition(Point3D(0.0, 0.0, 0.0)); //ball at center
    std::map<std::string, std::string> params;
    params["POI"] = "ball";

    EXPECT_FALSE(isOwnRobotSecondClosestToPOI(params));
}

TEST_F(PositionsTest, ownRobotFurthestFromPOIball)
{
    //initalize extra parameters;
    robotStore::getInstance().clear();
    robotStore::getInstance().addOwnRobot(teamplay::robot(2, treeEnum::DEFENDER_MAIN, Position2D(0.0, -9.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(1, treeEnum::R_GOALKEEPER, Position2D(0.0, -8.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(3, treeEnum::ATTACKER_MAIN, Position2D(0.0, 5.0, 0.0), Velocity2D()));
    ballStore::getBall().setPosition(Point3D(0.0, 0.0, 0.0)); //ball at center
    std::map<std::string, std::string> params;
    params["POI"] = "ball";

    EXPECT_TRUE(isOwnRobotFurthestFromPOI(params));
}

TEST_F(PositionsTest, ownRobotNotFurthestFromPOIball)
{
    //initalize extra parameters;
    robotStore::getInstance().clear();
    robotStore::getInstance().addOwnRobot(teamplay::robot(2, treeEnum::DEFENDER_MAIN, Position2D(0.0, -1.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(1, treeEnum::R_GOALKEEPER, Position2D(0.0, -8.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(3, treeEnum::ATTACKER_MAIN, Position2D(0.0, 5.0, 0.0), Velocity2D()));
    ballStore::getBall().setPosition(Point3D(0.0, 0.0, 0.0)); //ball at center
    std::map<std::string, std::string> params;
    params["POI"] = "ball";

    EXPECT_FALSE(isOwnRobotFurthestFromPOI(params));
}

TEST_F(PositionsTest, ownRobotSetpieceDefenderAssistball1)
{
    //initalize extra parameters;
    robotStore::getInstance().clear();
    robotStore::getInstance().addOwnRobot(teamplay::robot(4, treeEnum::DEFENDER_ASSIST, Position2D(0.0, -6.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(1, treeEnum::R_GOALKEEPER, Position2D(0.0, -8.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(3, treeEnum::ATTACKER_MAIN, Position2D(1.5, 1.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(5, treeEnum::ATTACKER_ASSIST, Position2D(0.0, 6.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(2, treeEnum::DEFENDER_MAIN, Position2D(2.0, 2.5, 0.0), Velocity2D()));
    ballStore::getBall().setPosition(Point3D(2.0, 2.0, 0.0)); //ball location
    std::map<std::string, std::string> params;
    params["POI"] = "ball";

    EXPECT_TRUE(isOwnRobotSetpieceDefenderAssist(params));
}

TEST_F(PositionsTest, ownRobotSetpieceDefenderAssistball2)
{
    //initalize extra parameters;
    robotStore::getInstance().clear();
    robotStore::getInstance().addOwnRobot(teamplay::robot(4, treeEnum::DEFENDER_ASSIST, Position2D(0.0, -6.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(1, treeEnum::R_GOALKEEPER, Position2D(0.0, -8.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(3, treeEnum::ATTACKER_MAIN, Position2D(1.5, -1.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(5, treeEnum::ATTACKER_ASSIST, Position2D(0.0, 6.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(2, treeEnum::DEFENDER_MAIN, Position2D(2.0, -2.5, 0.0), Velocity2D()));
    ballStore::getBall().setPosition(Point3D(2.0, -2.0, 0.0)); //ball location
    std::map<std::string, std::string> params;
    params["POI"] = "ball";

    EXPECT_TRUE(isOwnRobotSetpieceDefenderAssist(params));
}

TEST_F(PositionsTest, ownRobotSetpieceDefenderAssistball3)
{
    //initalize extra parameters;
    robotStore::getInstance().clear();
    robotStore::getInstance().addOwnRobot(teamplay::robot(5, treeEnum::ATTACKER_ASSIST, Position2D(0.0, 6.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(1, treeEnum::R_GOALKEEPER, Position2D(0.0, -8.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(3, treeEnum::ATTACKER_MAIN, Position2D(1.5, 1.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(4, treeEnum::DEFENDER_ASSIST, Position2D(0.0, -6.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(2, treeEnum::DEFENDER_MAIN, Position2D(2.0, 2.5, 0.0), Velocity2D()));
    ballStore::getBall().setPosition(Point3D(2.0, 2.0, 0.0)); //ball location
    std::map<std::string, std::string> params;
    params["POI"] = "ball";

    EXPECT_FALSE(isOwnRobotSetpieceDefenderAssist(params));
}

TEST_F(PositionsTest, ownRobotSetpieceDefenderAssistball4)
{
    //initalize extra parameters;
    robotStore::getInstance().clear();
    robotStore::getInstance().addOwnRobot(teamplay::robot(5, treeEnum::ATTACKER_ASSIST, Position2D(0.0, 6.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(1, treeEnum::R_GOALKEEPER, Position2D(0.0, -8.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(3, treeEnum::ATTACKER_MAIN, Position2D(1.5, -1.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(4, treeEnum::DEFENDER_ASSIST, Position2D(0.0, -6.0, 0.0), Velocity2D()));
    robotStore::getInstance().addTeammate(teamplay::robot(2, treeEnum::DEFENDER_MAIN, Position2D(2.0, -2.5, 0.0), Velocity2D()));
    ballStore::getBall().setPosition(Point3D(2.0, -2.0, 0.0)); //ball location
    std::map<std::string, std::string> params;
    params["POI"] = "ball";

    EXPECT_FALSE(isOwnRobotSetpieceDefenderAssist(params));
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    //Enable tracing
    //teamplay::traceRedirect::getInstance().setAllTracesToStdout();

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
