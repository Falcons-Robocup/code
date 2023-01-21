// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
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

// =============================
// isPotentialOppAttackerPresent
// =============================

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


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    //Enable tracing
    //teamplay::traceRedirect::getInstance().setAllTracesToStdout();

    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
