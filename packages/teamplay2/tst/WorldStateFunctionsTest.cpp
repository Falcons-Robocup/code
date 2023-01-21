// Copyright 2021 Erik Kouters (Falcons)
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
#include "int/WorldStateFunctions.hpp"
#include "int/stores/ConfigurationStore.hpp"
#include "int/stores/ObstacleStore.hpp"
#include "int/stores/RobotStore.hpp"

#include <math.h>

using namespace ::testing;
using namespace teamplay;


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
    std::vector<Obstacle> obstacles;

    // set other bot location(s) and stuff
    teamplay::ObstacleStore::getInstance().clear();
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(0.0, 6.0, 0.0)));
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(0.0, 4.0, 0.0)));

    // Execution
    Point2D targetPos = Point2D(0.0, 5.0);
    Point2D robotPos = Point2D(0.0, 0.0);
    WorldStateFunctions::getInstance().getObstructingObstaclesInPath(robotPos, targetPos, radiusBall, obstacles);

    // Verification
    EXPECT_TRUE(obstacles.size() == 1);
    EXPECT_FLOAT_EQ(obstacles.at(0).getPosition().getX(), 0.0);
    EXPECT_FLOAT_EQ(obstacles.at(0).getPosition().getY(), 4.0);

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
    std::vector<Obstacle> obstacles;

    // set othEr bot location(s) and stuff
    teamplay::ObstacleStore::getInstance().clear();
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(0.0, 6.0, 0.0)));
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(0.0, 5.0, 0.0)));

    // Execution
    Point2D targetPos = Point2D(0.0, 5.0);
    Point2D robotPos = Point2D(0.0, 0.0);
    WorldStateFunctions::getInstance().getObstructingObstaclesInPath(robotPos, targetPos, radiusBall, obstacles);

    // Verification
    EXPECT_TRUE(obstacles.size() == 1);
    EXPECT_FLOAT_EQ(obstacles.at(0).getPosition().getX(), 0.0);
    EXPECT_FLOAT_EQ(obstacles.at(0).getPosition().getY(), 5.0);
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
    std::vector<Obstacle> obstacles;

    // set other bot location(s) and stuff
    teamplay::ObstacleStore::getInstance().clear();
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(0.0, 6.0, 0.0)));
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(0.0, 4.0, 0.0)));

    // Execution
    Point2D targetPos = Point2D(0.0, 7.0);
    Point2D robotPos = Point2D(0.0, 0.0);
    WorldStateFunctions::getInstance().getObstructingObstaclesInPath(robotPos, targetPos, radiusBall, obstacles);

    // Verification
    EXPECT_TRUE(obstacles.size() == 2);
    EXPECT_FLOAT_EQ(obstacles.at(0).getPosition().getX(), 0.0);
    EXPECT_FLOAT_EQ(obstacles.at(0).getPosition().getY(), 4.0);
    EXPECT_FLOAT_EQ(obstacles.at(1).getPosition().getX(), 0.0);
    EXPECT_FLOAT_EQ(obstacles.at(1).getPosition().getY(), 6.0);
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
    std::vector<Obstacle> obstacles;

    // Clean the robot store and define one teammate
    teamplay::RobotStore::getInstance().clear();
    teamplay::RobotStore::getInstance().addTeammate(teamplay::Robot(2, RoleEnum::DEFENDER_MAIN, geometry::Pose2D(0.0, 5.0, 0.0), geometry::Velocity2D()));

    // set other bot location(s) and stuff
    teamplay::ObstacleStore::getInstance().clear();
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(0.0, 6.0, 0.0)));

    // Execution
    Point2D targetPos = Point2D(0.0, 5.0);
    Point2D robotPos = Point2D(0.0, 0.0);
    WorldStateFunctions::getInstance().getObstructingObstaclesInPath(robotPos, targetPos, radiusBall, obstacles);

    // Verification
    EXPECT_TRUE(obstacles.size() == 0);
}


//TEST(TestSuiteWorldStateFunctionsisPathToBallBlocked, PathBlocked_test1)
//{
//    // Setup
//    // Update WorldModel administration
//    // TESTCASE: OWN @0,0,
//    // Bal @0,6
//    // Bot1 @0,4 --> robot is obstructing path ball<>robot
//    // Radius ball = 0.125
//
//    std::vector<robotLocation> obstacles;
//
//    ConfigTeamplay config;
//    config.shooting.shootPathWidth = 0.25;
//    teamplay::configurationStore::getConfiguration().update(config);
//
//    teamplay::robotStore::getInstance().clear();
//    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, geometry::Pose2D(0.0, 0.0, 0.0), Velocity2D()));
//
//    // set ball location
//    teamplay::ballStore::getBall().setPosition(Point3D(0.0, 6.0, 0.0));
//
//    // set other bot location(s) and stuff
//    teamplay::obstacleStore::getInstance().clear();
//    teamplay::obstacleStore::getInstance().addObstacle(obstacle(geometry::Pose2D(0.0, 4.0, 0.0)));
//
//    // Execution
//    std::map<std::string, std::string> params;
//    EXPECT_TRUE(isPathToBallBlocked(params));
//}
//
//TEST(TestSuiteWorldStateFunctionsisPathToBallBlocked, PathBlocked_test2)
//{
//    // Setup
//    // Update WorldModel administration
//    // TESTCASE: OWN @0,0,
//    // Bal @0,6
//    // Bot1 @3,3 --> robot is NOT obstructing path ball<>robot
//    // Radius ball = 0.125
//
//    std::vector<robotLocation> obstacles;
//
//    ConfigTeamplay config;
//    config.shooting.shootPathWidth = 0.25;
//    teamplay::configurationStore::getConfiguration().update(config);
//
//    teamplay::robotStore::getInstance().clear();
//
//    teamplay::robotStore::getInstance().clear();
//    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, geometry::Pose2D(0.0, 0.0, 0.0), Velocity2D()));
//
//    // set ball location
//    teamplay::ballStore::getBall().setPosition(Point3D(0.0, 6.0, 0.0));
//
//    // set other bot location(s) and stuff
//    teamplay::obstacleStore::getInstance().clear();
//    teamplay::obstacleStore::getInstance().addObstacle(obstacle(geometry::Pose2D(3.0, 3.0, 0.0)));
//
//    // Execution
//    std::map<std::string, std::string> params;
//    EXPECT_FALSE(isPathToBallBlocked(params));
//}
//
//TEST(TestSuiteWorldStateFunctionsisOpponentHalfReachable, OpponentHalfReachable_test1)
//{
//    // Setup
//    // Update WorldModel administration
//    // TESTCASE: OWN @0,-0.1,
//    // Ball same as own.picked up at (0, 0.1)
//
//    teamplay::robotStore::getInstance().clear();
//    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, geometry::Pose2D(0.0, -0.1, 0.0), Velocity2D()));
//
//    // set ball location
//    teamplay::ballStore::getBall().setPosition(Point3D(0.0, 0.1, 0.0));
//    teamplay::ballStore::getBall().setPositionClaimed(Point3D(0.0, 0.1, 0.0));
//
//    // Execution
//    std::map<std::string, std::string> params;
//    EXPECT_TRUE(isOpponentHalfReachable(params));
//}
//
//TEST(TestSuiteWorldStateFunctionsisOpponentHalfReachable, OpponentHalfReachable_test2)
//{
//    // Setup
//    // Update WorldModel administration
//    // TESTCASE: OWN @0,-1.5,
//    // Ball same as own.picked up at (0, -1.5)
//
//    teamplay::robotStore::getInstance().clear();
//    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::ATTACKER_MAIN, geometry::Pose2D(0.0, -1.5, 0.0), Velocity2D()));
//
//    // set ball location
//    teamplay::ballStore::getBall().setPosition(Point3D(0.0, 1.5, 0.0));
//    teamplay::ballStore::getBall().setPositionClaimed(Point3D(0.0, 1.5, 0.0));
//
//    // Execution
//    std::map<std::string, std::string> params;
//    EXPECT_TRUE(isOpponentHalfReachable(params));
//}
//
//TEST(TestSuiteWorldStateFunctionsClosestAttackerToOppGoal, closestAttToOppGoal_test1)
//{
//    // Setup
//    // Update WorldModel administration
//    // TESTCASE: OWN @0,0,
//    // Member1 @0,5
//    // Member2 @0,7
//
//    teamplay::robotStore::getInstance().clear();
//
//    teamplay::robotStore::getInstance().addOwnRobot(teamplay::robot(1, treeEnum::R_GOALKEEPER, geometry::Pose2D(0.0, 0.0, 0.0), Velocity2D()));
//    teamplay::robotStore::getInstance().addTeammate(teamplay::robot(2, treeEnum::ATTACKER_MAIN, geometry::Pose2D(0.0, 5.0, 0.0), Velocity2D()));
//    teamplay::robotStore::getInstance().addTeammate(teamplay::robot(3, treeEnum::ATTACKER_ASSIST, geometry::Pose2D(0.0, 7.0, 0.0), Velocity2D()));
//
//    // Execution
//    auto result = cWorldStateFunctions::getInstance().getClosestAttackerToOpponentGoal();
//
//    // Verification
//    ASSERT_TRUE((bool)result);
//    EXPECT_EQ(0.0, result->getLocation().x) << "Wrong x position";
//    EXPECT_EQ(7.0, result->getLocation().y) << "Wrong y position";
//}



// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    //Enable tracing
    //teamplay::traceRedirect::getInstance().setAllTracesToStdout();

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
