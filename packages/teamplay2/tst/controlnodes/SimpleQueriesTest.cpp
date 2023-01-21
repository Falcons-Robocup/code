// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Bring in gtest
#include "gtest/gtest.h"

/* Include trace utility */
#include "tracing.hpp"

// Bring in my package's API, which is what I'm testing
#include "int/controlnodes/SimpleQueries.hpp"
#include "int/stores/ObstacleStore.hpp"
#include "int/stores/RobotStore.hpp"

#include "cEnvironmentField.hpp"

#include <math.h>

using namespace ::testing;
using namespace teamplay;



// ======================
// doesTeamHaveBall
// ======================

TEST(WorldStateFunctions, doesTeamHaveBall_WhenBallPossessionThisRobot_ShouldReturnTrue)
{
    // Arrange
    teamplay::RobotStore::getInstance().clear();

    // Update administration that own robot has the ball
    auto own_robot = teamplay::Robot();
    own_robot.claimsBallPossession();
    teamplay::RobotStore::getInstance().addOwnRobot(own_robot);

    auto teammate = teamplay::Robot();
    teammate.losesBallPossession();
    teamplay::RobotStore::getInstance().addTeammate(teammate);

    // Act
    bool result = doesTeamHaveBall();

    // Assert
    EXPECT_TRUE(result);
}

TEST(WorldStateFunctions, doesTeamHaveBall_WhenBallPossessionOtherTeamRobot_ShouldReturnTrue)
{
    // Arrange
    teamplay::RobotStore::getInstance().clear();

    // Update administration that own robot does *not* have the ball
    auto own_robot = teamplay::Robot();
    own_robot.losesBallPossession();
    teamplay::RobotStore::getInstance().addOwnRobot(own_robot);

    auto teammate = teamplay::Robot();
    teammate.claimsBallPossession();
    teamplay::RobotStore::getInstance().addTeammate(teammate);

    // Act
    bool result = doesTeamHaveBall();

    // Assert
    EXPECT_TRUE(result);
}

TEST(WorldStateFunctions, doesTeamHaveBall_WhenFieldHasBall_ShouldReturnFalse)
{
    // Arrange
    teamplay::RobotStore::getInstance().clear();

    // Update administration that field has the ball
    auto own_robot = teamplay::Robot();
    own_robot.losesBallPossession();
    teamplay::RobotStore::getInstance().addOwnRobot(own_robot);

    auto teammate = teamplay::Robot();
    teammate.losesBallPossession();
    teamplay::RobotStore::getInstance().addTeammate(teammate);

    // Act
    bool result = doesTeamHaveBall();

    // Assert
    EXPECT_FALSE(result);
}

// ======================
// doesOwnRobotHaveBall
// ======================

TEST(WorldStateFunctions, doesOwnRobotHaveBall_WhenBallPossessionThisRobot_ShouldReturnTrue)
{
    // Arrange
    teamplay::RobotStore::getInstance().clear();

    // Update administration that own robot has the ball
    auto own_robot = teamplay::Robot();
    own_robot.claimsBallPossession();
    teamplay::RobotStore::getInstance().addOwnRobot(own_robot);

    auto teammate = teamplay::Robot();
    teammate.losesBallPossession();
    teamplay::RobotStore::getInstance().addTeammate(teammate);

    // Act
    bool result = doesOwnRobotHaveBall();

    // Assert
    EXPECT_TRUE(result);
}

TEST(WorldStateFunctions, doesOwnRobotHaveBall_WhenBallPossessionOtherTeamRobot_ShouldReturnFalse)
{
    // Arrange
    teamplay::RobotStore::getInstance().clear();

    // Update administration that other team robot has the ball
    auto own_robot = teamplay::Robot();
    own_robot.losesBallPossession();
    teamplay::RobotStore::getInstance().addOwnRobot(own_robot);

    auto teammate = teamplay::Robot();
    teammate.claimsBallPossession();
    teamplay::RobotStore::getInstance().addTeammate(teammate);

    // Act
    bool result = doesOwnRobotHaveBall();

    // Assert
    EXPECT_FALSE(result);
}

// ==================================
// isShortTurnToGoalBlockedByOpponent
// isLongTurnToGoalBlockedByOpponent
// ==================================

TEST(areTurnsToGoalBlockedByOpponent, Test1)
{
    teamplay::RobotStore::getInstance().clear();
    teamplay::RobotStore::getInstance().addOwnRobot(teamplay::Robot(1, RoleEnum::ATTACKER_MAIN, geometry::Pose2D(-4.0, 2.0, M_PI), geometry::Velocity2D()));

    teamplay::ObstacleStore::getInstance().clear();
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-4.0, 0.4, 0.0)));
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-2.5, 3.5, 0.0)));
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-1.0, 7.0, 0.0)));

    EXPECT_FALSE(isShortTurnToGoalBlockedByOpponent());
    EXPECT_FALSE(isLongTurnToGoalBlockedByOpponent());
}

TEST(areTurnsToGoalBlockedByOpponent, Test2)
{
    teamplay::RobotStore::getInstance().clear();
    teamplay::RobotStore::getInstance().addOwnRobot(teamplay::Robot(1, RoleEnum::ATTACKER_MAIN, geometry::Pose2D(-4.0, 2.0, M_PI), geometry::Velocity2D()));

    teamplay::ObstacleStore::getInstance().clear();
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-4.0, 0.4, 0.0)));
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-4.0, 2.9, 0.0)));
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-1.0, 7.0, 0.0)));

    EXPECT_TRUE(isShortTurnToGoalBlockedByOpponent());
    EXPECT_FALSE(isLongTurnToGoalBlockedByOpponent());
}

TEST(areTurnsToGoalBlockedByOpponent, Test3)
{
    teamplay::RobotStore::getInstance().clear();
    teamplay::RobotStore::getInstance().addOwnRobot(teamplay::Robot(1, RoleEnum::ATTACKER_MAIN, geometry::Pose2D(-4.0, 2.0, 0.0), geometry::Velocity2D()));

    teamplay::ObstacleStore::getInstance().clear();
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-4.0, 0.4, 0.0)));
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-4.0, 2.9, 0.0)));
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-1.0, 7.0, 0.0)));

    EXPECT_FALSE(isShortTurnToGoalBlockedByOpponent());
    EXPECT_TRUE(isLongTurnToGoalBlockedByOpponent());
}

TEST(areTurnsToGoalBlockedByOpponent, Test4)
{
    teamplay::RobotStore::getInstance().clear();
    teamplay::RobotStore::getInstance().addOwnRobot(teamplay::Robot(1, RoleEnum::ATTACKER_MAIN, geometry::Pose2D(-4.0, 2.0, M_PI), geometry::Velocity2D()));

    teamplay::ObstacleStore::getInstance().clear();
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-4.0, 1.1, 0.0)));
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-4.0, 2.9, 0.0)));
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-1.0, 7.0, 0.0)));

    EXPECT_TRUE(isShortTurnToGoalBlockedByOpponent());
    EXPECT_TRUE(isLongTurnToGoalBlockedByOpponent());
}

TEST(areTurnsToGoalBlockedByOpponent, Test5)
{
    teamplay::RobotStore::getInstance().clear();
    teamplay::RobotStore::getInstance().addOwnRobot(teamplay::Robot(1, RoleEnum::ATTACKER_MAIN, geometry::Pose2D(0.0, 5.0, (1.4 * M_PI)), geometry::Velocity2D()));

    teamplay::ObstacleStore::getInstance().clear();
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-0.5, 7.0, 0.0)));
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(0.5, 7.0, 0.0)));

    EXPECT_FALSE(isShortTurnToGoalBlockedByOpponent());
    EXPECT_FALSE(isLongTurnToGoalBlockedByOpponent());
}

TEST(areTurnsToGoalBlockedByOpponent, Test6)
{
    teamplay::RobotStore::getInstance().clear();
    teamplay::RobotStore::getInstance().addOwnRobot(teamplay::Robot(1, RoleEnum::ATTACKER_MAIN, geometry::Pose2D(0.0, 5.0, (1.4 * M_PI)), geometry::Velocity2D()));

    teamplay::ObstacleStore::getInstance().clear();
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-0.5, 5.5, 0.0)));
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(0.5, 7.0, 0.0)));

    EXPECT_TRUE(isShortTurnToGoalBlockedByOpponent());
    EXPECT_FALSE(isLongTurnToGoalBlockedByOpponent());
}

TEST(areTurnsToGoalBlockedByOpponent, Test7)
{
    teamplay::RobotStore::getInstance().clear();
    teamplay::RobotStore::getInstance().addOwnRobot(teamplay::Robot(1, RoleEnum::ATTACKER_MAIN, geometry::Pose2D(0.0, 5.0, (1.4 * M_PI)), geometry::Velocity2D()));

    teamplay::ObstacleStore::getInstance().clear();
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-0.5, 7.0, 0.0)));
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(0.5, 5.5, 0.0)));

    EXPECT_FALSE(isShortTurnToGoalBlockedByOpponent());
    EXPECT_TRUE(isLongTurnToGoalBlockedByOpponent());
}

TEST(areTurnsToGoalBlockedByOpponent, Test8)
{
    cEnvironmentField::getInstance().loadConfig("cEnvironment12x18");

    teamplay::RobotStore::getInstance().clear();
    teamplay::RobotStore::getInstance().addOwnRobot(teamplay::Robot(1, RoleEnum::ATTACKER_MAIN, geometry::Pose2D(3.0, 7.0, (2.0 * M_PI)), geometry::Velocity2D()));

    teamplay::ObstacleStore::getInstance().clear();
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(2.5, 7.5, 0.0)));

    EXPECT_TRUE(isShortTurnToGoalBlockedByOpponent());
    EXPECT_FALSE(isLongTurnToGoalBlockedByOpponent());
}

TEST(areTurnsToGoalBlockedByOpponent, Test9)
{
    teamplay::RobotStore::getInstance().clear();
    teamplay::RobotStore::getInstance().addOwnRobot(teamplay::Robot(1, RoleEnum::ATTACKER_MAIN, geometry::Pose2D(3.0, 7.0, (1.5 * M_PI)), geometry::Velocity2D()));

    teamplay::ObstacleStore::getInstance().clear();
    teamplay::ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(2.5, 7.0, 0.0)));

    EXPECT_TRUE(isShortTurnToGoalBlockedByOpponent());
    EXPECT_FALSE(isLongTurnToGoalBlockedByOpponent());
}



// ==================================
// isLowestActiveRobotID
// isSecondHighestActiveRobotID
// isThirdHighestActiveRobotID
// isHighestActiveRobotID
// ==================================

class TeamTest : public Test
{
public:
    TeamTest()
{
        RobotStore::getInstance().clear();
        RobotStore::getInstance().addOwnRobot(Robot(1, RoleEnum::GOALKEEPER, geometry::Pose2D(), geometry::Velocity2D()));
        RobotStore::getInstance().addTeammate(Robot(2, RoleEnum::DEFENDER_MAIN, geometry::Pose2D(), geometry::Velocity2D()));
        RobotStore::getInstance().addTeammate(Robot(3, RoleEnum::ATTACKER_MAIN, geometry::Pose2D(), geometry::Velocity2D()));
        RobotStore::getInstance().addTeammate(Robot(4, RoleEnum::DEFENDER_ASSIST, geometry::Pose2D(), geometry::Velocity2D()));
        RobotStore::getInstance().addTeammate(Robot(5, RoleEnum::ATTACKER_ASSIST, geometry::Pose2D(), geometry::Velocity2D()));
}
};

TEST_F(TeamTest, ownRobotIsLowestActiveRobot)
{
    EXPECT_TRUE(isLowestActiveRobotID());
    EXPECT_FALSE(isSecondHighestActiveRobotID());
    EXPECT_FALSE(isThirdHighestActiveRobotID());
    EXPECT_FALSE(isHighestActiveRobotID());
}

TEST_F(TeamTest, ownRobotIsHighestActiveRobot)
{
    RobotStore::getInstance().exchangeOwnRobotWith(5);

    EXPECT_FALSE(isLowestActiveRobotID());
    EXPECT_FALSE(isSecondHighestActiveRobotID());
    EXPECT_FALSE(isThirdHighestActiveRobotID());
    EXPECT_TRUE(isHighestActiveRobotID());

    RobotStore::getInstance().undoExchange();
}

TEST_F(TeamTest, ownRobotIsSecondHighestActiveRobot)
{
    RobotStore::getInstance().exchangeOwnRobotWith(4);

    EXPECT_FALSE(isLowestActiveRobotID());
    EXPECT_TRUE(isSecondHighestActiveRobotID());
    EXPECT_FALSE(isThirdHighestActiveRobotID());
    EXPECT_FALSE(isHighestActiveRobotID());

    RobotStore::getInstance().undoExchange();
}

TEST_F(TeamTest, ownRobotIsThirdHighestActiveRobot)
{
    RobotStore::getInstance().exchangeOwnRobotWith(3);

    EXPECT_FALSE(isLowestActiveRobotID());
    EXPECT_FALSE(isSecondHighestActiveRobotID());
    EXPECT_TRUE(isThirdHighestActiveRobotID());
    EXPECT_FALSE(isHighestActiveRobotID());

    RobotStore::getInstance().undoExchange();
}

TEST_F(TeamTest, ownRobotIsOnlyActiveRobot)
{
    EXPECT_FALSE(isOnlyActiveRobotID());

    RobotStore::getInstance().clear();
    RobotStore::getInstance().addOwnRobot(Robot(1, RoleEnum::GOALKEEPER, geometry::Pose2D(), geometry::Velocity2D()));
    EXPECT_TRUE(isOnlyActiveRobotID());
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    //Enable tracing
    //teamplay::traceRedirect::getInstance().setAllTracesToStdout();

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
