 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cWorldStateFunctionsTest.cpp
 *
 *  Created on: Oct 13, 2015
 *      Author: Ivo Matthijssen
 */
// Bring in gtest
#include <gtest/gtest.h>

#include "area2D.hpp"

/* Include trace utility */
#include "int/utilities/trace.hpp"

// Bring in my package's API, which is what I'm testing
#include "int/cObjectPath.hpp"
#include "int/cWorldStateFunctions.hpp"
#include "int/cWorldModelInterface.hpp"
#include "int/utilities/trace.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/stores/fieldDimensionsStore.hpp"
#include "int/types/fieldDimensions.hpp"
#include "int/stores/teamMatesStore.hpp"

#include <math.h>

// ======================
// doesTeamHaveBall
// ======================

TEST(WorldStateFunctions, doesTeamHaveBall_WhenBallPossessionThisRobot_ShouldReturnTrue)
{
    // Arrange
    // Update WorldModel administration that own robot has the ball
    ballPossession_struct_t ballPossession;
    ballPossession.robotID = 0;  // own robot id = 0
    ballPossession.possessionType = ballPossessionEnum::TEAMMEMBER;
    cWorldModelInterface::getInstance().setBallPossession(ballPossession);

    // Act
    bool result = doesTeamHaveBall();

    // Assert
    EXPECT_TRUE(result);
}

TEST(WorldStateFunctions, doesTeamHaveBall_WhenBallPossessionOtherTeamRobot_ShouldReturnTrue)
{
    // Arrange
    // Update WorldModel administration that own robot does *not* have the ball
    ballPossession_struct_t ballPossession;
    ballPossession.robotID = 1;  // own robot id = 0
    ballPossession.possessionType = ballPossessionEnum::TEAMMEMBER;
    cWorldModelInterface::getInstance().setBallPossession(ballPossession);

    // Act
    bool result = doesTeamHaveBall();

    // Assert
    EXPECT_TRUE(result);
}

TEST(WorldStateFunctions, doesTeamHaveBall_WhenFieldHasBall_ShouldReturnFalse)
{
    // Arrange
    // Update WorldModel administration that field has the ball
    ballPossession_struct_t ballPossession;
    ballPossession.possessionType = ballPossessionEnum::FIELD;
    cWorldModelInterface::getInstance().setBallPossession(ballPossession);

    // Act
    bool result = doesTeamHaveBall();

    // Assert
    EXPECT_FALSE(result);
}

TEST(WorldStateFunctions, doesTeamHaveBall_WhenBallPossessionINVALID_ShouldReturnFalse)
{
    // Arrange
    // Update WorldModel administration that field has the ball
    ballPossession_struct_t ballPossession;
    ballPossession.possessionType = ballPossessionEnum::INVALID;
    cWorldModelInterface::getInstance().setBallPossession(ballPossession);

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
    // Update WorldModel administration that own robot has the ball
    ballPossession_struct_t ballPossession;
    ballPossession.robotID = 0;  // own robot id = 0
    ballPossession.possessionType = ballPossessionEnum::TEAMMEMBER;
    cWorldModelInterface::getInstance().setBallPossession(ballPossession);

    // Act
    bool result = doesOwnRobotHaveBall();

    // Assert
    EXPECT_TRUE(result);
}

TEST(WorldStateFunctions, doesOwnRobotHaveBall_WhenBallPossessionOtherTeamRobot_ShouldReturnFalse)
{
    // Arrange
    // Update WorldModel administration that other team robot has the ball
    ballPossession_struct_t ballPossession;
    ballPossession.robotID = 1;  // own robot id = 0
    ballPossession.possessionType = ballPossessionEnum::TEAMMEMBER;
    cWorldModelInterface::getInstance().setBallPossession(ballPossession);

    // Act
    bool result = doesOwnRobotHaveBall();

    // Assert
    EXPECT_FALSE(result);
}

// ======================
// isBallInOwnPenaltyArea
// ======================

TEST(WorldStateFunctions, isBallInOwnPenaltyArea_WhenBallLocationInOwnHalf_ShouldReturnTrue)
{
    // Arrange
    // Update WorldModel administration that the ball is in own half
    ballLocation lastBallLocation;
    lastBallLocation.position.x = 1.0;
    lastBallLocation.position.y = 1.0;

    ballLocations newBallLocations;
    ballLocation newLocation;

    // Get a coordinate in the penalty area
    Area2D penaltyArea = teamplay::fieldDimensionsStore::getFieldDimensions().getArea(teamplay::fieldArea::OWN_PENALTYAREA);
    newLocation.position.x = (penaltyArea.ll.x + penaltyArea.ur.x) / 2.0;
    newLocation.position.y = (penaltyArea.ll.y + penaltyArea.ur.y) / 2.0;

    newBallLocations.push_back(newLocation);
    cWorldModelInterface::getInstance().setBallLocation(newBallLocations, lastBallLocation);

    // Act
    bool result = isBallInOwnPenaltyArea();

    // Assert
    EXPECT_TRUE(result);
}

TEST(WorldStateFunctions, isBallInOwnPenaltyArea_WhenBallLocationInOpponentHalf_ShouldReturnFalse)
{
    // Arrange
    // Update WorldModel administration that the ball is in own half
    ballLocation lastBallLocation;
    lastBallLocation.position.x = 1.0;
    lastBallLocation.position.y = 1.0;

    ballLocations newBallLocations;
    ballLocation newLocation;

    // Get a coordinate in the penalty area
    Area2D penaltyArea = teamplay::fieldDimensionsStore::getFieldDimensions().getArea(teamplay::fieldArea::OPP_PENALTYAREA);
    newLocation.position.x = (penaltyArea.ll.x + penaltyArea.ur.x) / 2.0;
    newLocation.position.y = (penaltyArea.ll.y + penaltyArea.ur.y) / 2.0;

    newBallLocations.push_back(newLocation);
    cWorldModelInterface::getInstance().setBallLocation(newBallLocations, lastBallLocation);

    // Act
    bool result = isBallInOwnPenaltyArea();

    // Assert
    EXPECT_FALSE(result);
}

TEST(WorldStateFunctions, isBallInOwnPenaltyArea_WhenNoBallSeen_ShouldReturnFalse)
{
    // Arrange
    ballLocation lastBallLocation;
    lastBallLocation.position.x = 1.0;
    lastBallLocation.position.y = 1.0;

    ballLocations newBallLocations; // If this is empty, no balls seen.
    cWorldModelInterface::getInstance().setBallLocation(newBallLocations, lastBallLocation);

    // Act
    bool result = isBallInOwnPenaltyArea();

    // Assert
    EXPECT_FALSE(result);
}

// ======================
// isBallInOwnPenaltyArea
// ======================

TEST(WorldStateFunctions, isBallInOpponentPenaltyArea_WhenBallLocationInOpponentHalf_ShouldReturnTrue)
{
    // Arrange
    // Update WorldModel administration that the ball is in own half
    ballLocation lastBallLocation;
    lastBallLocation.position.x = 1.0;
    lastBallLocation.position.y = 1.0;

    ballLocations newBallLocations;
    ballLocation newLocation;

    // Get a coordinate in the penalty area
    Area2D penaltyArea = teamplay::fieldDimensionsStore::getFieldDimensions().getArea(teamplay::fieldArea::OPP_PENALTYAREA);
    newLocation.position.x = (penaltyArea.ll.x + penaltyArea.ur.x) / 2.0;
    newLocation.position.y = (penaltyArea.ll.y + penaltyArea.ur.y) / 2.0;

    newBallLocations.push_back(newLocation);
    cWorldModelInterface::getInstance().setBallLocation(newBallLocations, lastBallLocation);

    // Act
    bool result = isBallInOpponentPenaltyArea();

    // Assert
    EXPECT_TRUE(result);
}

TEST(WorldStateFunctions, isBallInOpponentPenaltyArea_WhenBallLocationInOwnHalf_ShouldReturnFalse)
{
    // Arrange
    // Update WorldModel administration that the ball is in own half
    ballLocation lastBallLocation;
    lastBallLocation.position.x = 1.0;
    lastBallLocation.position.y = 1.0;

    ballLocations newBallLocations;
    ballLocation newLocation;

    // Get a coordinate in the penalty area
    Area2D penaltyArea = teamplay::fieldDimensionsStore::getFieldDimensions().getArea(teamplay::fieldArea::OWN_PENALTYAREA);
    newLocation.position.x = (penaltyArea.ll.x + penaltyArea.ur.x) / 2.0;
    newLocation.position.y = (penaltyArea.ll.y + penaltyArea.ur.y) / 2.0;

    newBallLocations.push_back(newLocation);
    cWorldModelInterface::getInstance().setBallLocation(newBallLocations, lastBallLocation);

    // Act
    bool result = isBallInOpponentPenaltyArea();

    // Assert
    EXPECT_FALSE(result);
}

TEST(WorldStateFunctions, isBallInOpponentPenaltyArea_WhenNoBallSeen_ShouldReturnFalse)
{
    // Arrange
    ballLocation lastBallLocation;
    lastBallLocation.position.x = 1.0;
    lastBallLocation.position.y = 1.0;

    ballLocations newBallLocations; // If this is empty, no balls seen.
    cWorldModelInterface::getInstance().setBallLocation(newBallLocations, lastBallLocation);

    // Act
    bool result = isBallInOpponentPenaltyArea();

    // Assert
    EXPECT_FALSE(result);
}

// ======================
// getClosestTeammember
// ======================

TEST(WorldStateFunctions, getClosestTeammember_WhenNoTeammembersSet_ShouldReturnFalse)
{
    // Arrange
    // No team members set

    double x,y;
    // Act
    bool result = cWorldStateFunctions::getInstance().getClosestTeammember(x,y, false);

    // Assert
    EXPECT_FALSE(result);
}


TEST(WorldStateFunctions, getClosestTeammember_WhenNoOtherRobotsPresent_ShouldReturnFalse)
{
    // Arrange
    std::vector<robotNumber> activeRobots; // No other robots. Note: Do not add self (id 0)
    robotLocations teammembers; // No other robots.

    cWorldModelInterface::getInstance().setTeammembers(teammembers, activeRobots); // Set empty team members

    double x,y;
    // Act
    bool result = cWorldStateFunctions::getInstance().getClosestTeammember(x,y, false);

    // Assert
    EXPECT_FALSE(result);
}

// Test fixture that sets up a second team member with id 8 and position x=6.0, y=7.0
// Sets own position at x=1.0,y=2.0
class WorldStateFunctions_WhenOneOtherRobotPresent : public ::testing::Test {
 protected:

    WorldStateFunctions_WhenOneOtherRobotPresent() 
    {
        // Arrange
        std::vector<robotNumber> activeRobots;
        robotLocations teammembers;

        // Add robot with id 8.  Note: Do not add self (id 0)
        activeRobots.push_back(8); 

        // Supply location for robot 8
        robotLocation mate8;
        mate8.position = geometry::Pose2D(6.0, 7.0, 0.0);
        teammembers.insert(std::pair<robotNumber, robotLocation>(8, mate8));

        cWorldModelInterface::getInstance().setTeammembers(teammembers, activeRobots);

        // Set own location
        Position2D position(1.0, 2.0, 0.0);
        Velocity2D velocity(0.0, 0.0, 0.0);
        cWorldModelInterface::getInstance().setOwnRobot(position, velocity);
    }
};

TEST_F(WorldStateFunctions_WhenOneOtherRobotPresent, getClosestTeammember_ShouldReturnTrue)
{
    // Arrange
    // Done in test fixture

    double x,y;
    // Act
    bool result = cWorldStateFunctions::getInstance().getClosestTeammember(x,y, false);

    // Assert
    EXPECT_TRUE(result);
}

TEST_F(WorldStateFunctions_WhenOneOtherRobotPresent, getClosestTeammember_xValueSetToTeammatePosition)
{
    // Arrange
    // Done in test fixture

    double x,y;
    // Act
    bool result = cWorldStateFunctions::getInstance().getClosestTeammember(x,y, false);

    // Assert
    EXPECT_EQ(6.0, x);
}

TEST_F(WorldStateFunctions_WhenOneOtherRobotPresent, getClosestTeammember_yValueSetToTeammatePosition)
{
    // Arrange
    // Done in test fixture

    double x,y;
    // Act
    bool result = cWorldStateFunctions::getInstance().getClosestTeammember(x,y, false);

    // Assert
    EXPECT_EQ(7.0, y);
}

// Test fixture that sets up three other team members: 8 (x=6.0, y=7.0), 15 (x=1.0, y=3.0) and 20 (x=7.0, y=6.0)
// Sets own position at x=1.0,y=1.0
class WorldStateFunctions_WhenMultipleOtherRobotsPresent : public ::testing::Test {
 protected:

    WorldStateFunctions_WhenMultipleOtherRobotsPresent()
    {
        std::vector<robotNumber> activeRobots;
        robotLocations teammembers;

        // Add robots. Note: Do not add self (id 0)
        activeRobots.push_back(8); 
        activeRobots.push_back(15); 
        activeRobots.push_back(20); 

        // Supply locations
        robotLocation mate8;
        mate8.position = geometry::Pose2D(6.0, 7.0, 0.0);
        teammembers.insert(std::pair<robotNumber, robotLocation>(8, mate8));

        robotLocation mate15;
        mate15.position = geometry::Pose2D(1.0, 3.0, 0.0);
        teammembers.insert(std::pair<robotNumber, robotLocation>(15, mate15));

        robotLocation mate20;
        mate20.position = geometry::Pose2D(7.0, 6.0, 0.0);
        teammembers.insert(std::pair<robotNumber, robotLocation>(20, mate20));

        cWorldModelInterface::getInstance().setTeammembers(teammembers, activeRobots);

        // Set own location
        Position2D position(1.0, 1.0, 0.0);
        Velocity2D velocity(0.0, 0.0, 0.0);
        cWorldModelInterface::getInstance().setOwnRobot(position, velocity);
    }
};

TEST_F(WorldStateFunctions_WhenMultipleOtherRobotsPresent, getClosestTeammember_ShouldReturnTrue)
{
    // Arrange
    // Done in test fixture

    double x,y;
    // Act
    bool result = cWorldStateFunctions::getInstance().getClosestTeammember(x,y, false);

    // Assert
    EXPECT_TRUE(result);
}

TEST_F(WorldStateFunctions_WhenMultipleOtherRobotsPresent, getClosestTeammember_xValueSetToClosestTeammatePosition)
{
    // Arrange
    // Done in test fixture

    double x,y;
    // Act
    bool result = cWorldStateFunctions::getInstance().getClosestTeammember(x,y, false);

    // Assert
    EXPECT_EQ(1.0, x);
}

TEST_F(WorldStateFunctions_WhenMultipleOtherRobotsPresent, getClosestTeammember_yValueSetToClosestTeammatePosition)
{
    // Arrange
    // Done in test fixture

    double x,y;
    // Act
    bool result = cWorldStateFunctions::getInstance().getClosestTeammember(x,y, false);

    // Assert
    EXPECT_EQ(3.0, y);
}

// Test fixture that sets up one other team member at coordinate x=1.0,y=1.0
// Sets own position at x=1.0,y=1.0
class WorldStateFunctions_WhenTeamMemberAtExactSameLocation : public ::testing::Test {
 protected:

    WorldStateFunctions_WhenTeamMemberAtExactSameLocation()
    {
        std::vector<robotNumber> activeRobots;
        robotLocations teammembers;

        // Add robots. Note: Do not add self (id 0)
        activeRobots.push_back(8); 

        // Supply locations
        robotLocation mate8;
        mate8.position = geometry::Pose2D(1.0, 1.0, 0.0);
        teammembers.insert(std::pair<robotNumber, robotLocation>(8, mate8));

        cWorldModelInterface::getInstance().setTeammembers(teammembers, activeRobots);

        // Set own location
        Position2D position(1.0, 1.0, 0.0);
        Velocity2D velocity(0.0, 0.0, 0.0);
        cWorldModelInterface::getInstance().setOwnRobot(position, velocity);
    }
};

TEST_F(WorldStateFunctions_WhenTeamMemberAtExactSameLocation, getClosestTeammember_ShouldReturnTrue)
{
    // Arrange
    // Done in test fixture

    double x,y;
    // Act
    bool result = cWorldStateFunctions::getInstance().getClosestTeammember(x,y, false);

    // Assert
    EXPECT_TRUE(result);
}

TEST_F(WorldStateFunctions_WhenTeamMemberAtExactSameLocation, getClosestTeammember_xValueSetToClosestTeammatePosition)
{
    // Arrange
    // Done in test fixture

    double x,y;
    // Act
    bool result = cWorldStateFunctions::getInstance().getClosestTeammember(x,y, false);

    // Assert
    EXPECT_EQ(1.0, x);
}

TEST_F(WorldStateFunctions_WhenTeamMemberAtExactSameLocation, getClosestTeammember_yValueSetToClosestTeammatePosition)
{
    // Arrange
    // Done in test fixture

    double x,y;
    // Act
    bool result = cWorldStateFunctions::getInstance().getClosestTeammember(x,y, false);

    // Assert
    EXPECT_EQ(1.0, y);
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
	Position2D ownTestLocation(5.0, 5.0, 0.0);
	cWorldModelInterface::getInstance().setOwnLocation( ownTestLocation );

	// set other bot location(s) and stuff
    robotLocations opponents;
    // --R8
    robotLocation opp8;
	opp8.position = geometry::Pose2D(6.0, 6.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(8, opp8));
	// --R15
    robotLocation opp15;
	opp15.position = geometry::Pose2D(8.0, 8.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(15, opp15));
	// --R20
    robotLocation opp20;
	opp20.position = geometry::Pose2D(5.1, 5.4, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(20, opp20));

	cWorldModelInterface::getInstance().setOpponents(opponents);

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
    robotLocations opponents;
    // --R8
    robotLocation opp1;
	opp1.position = geometry::Pose2D(0.0, 1.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(1, opp1));
	// --R15
    robotLocation opp2;
	opp2.position = geometry::Pose2D(0.0, -1.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(2, opp2));

	cWorldModelInterface::getInstance().setOpponents(opponents);

    // Execution
	bool retval = multipleOpponentsOnOwnHalf();

	// Verification
	EXPECT_TRUE(retval == false);
}

TEST(TestSuiteWorldStateFunctionsMulptipleOpponentsOnOwnHalf, MultipleOpponentsOnOwnHalf_test2)
{
	// Setup
	// Update WorldModel administration
	// TESTCASE: Bot1 @ 0,-1, Bot 2 @ 0,-2, => result shoud be true, more than 1 opponent on own half

	// set other bot location(s) and stuff
    robotLocations opponents;
    // --R8
    robotLocation opp1;
	opp1.position = geometry::Pose2D(0.0, -1.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(1, opp1));
	// --R15
    robotLocation opp2;
	opp2.position = geometry::Pose2D(0.0, -2.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(2, opp2));

	cWorldModelInterface::getInstance().setOpponents(opponents);

    // Execution
	bool retval = multipleOpponentsOnOwnHalf();

	// Verification
	EXPECT_TRUE(retval == true);
}

TEST(TestSuiteWorldStateFunctionsIsOpponentWithinXMeterFromOwnGoal, IsOpponentWithinXMeterFromOwnGoal_test1)
{
	// Setup
	// Update WorldModel administration
	// TESTCASE: Bot1 @ 0,-8, --> result shoud be true, closestOpponent is within 4.5 meters from goal

	// set other bot location(s) and stuff
    robotLocations opponents;
    // --R1
    robotLocation opp1;
	opp1.position = geometry::Pose2D(0.0, -8.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(1, opp1));

	cWorldModelInterface::getInstance().setOpponents(opponents);

	// Verification
	EXPECT_TRUE(isOpponentWithinXMeterFromOwnGoal());
}

TEST(TestSuiteWorldStateFunctionsIsOpponentWithinXMeterFromOwnGoal, IsOpponentWithinXMeterFromOwnGoal_test2)
{
	// Setup
	// Update WorldModel administration
	// TESTCASE: Bot1 @ 0,-2, --> result shoud be false, closestOpponent is NOT within 4.5 meters from goal

	// set other bot location(s) and stuff
    robotLocations opponents;
    // --R1
    robotLocation opp1;
	opp1.position = geometry::Pose2D(0.0, -2.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(1, opp1));

	cWorldModelInterface::getInstance().setOpponents(opponents);

	// Verification
	EXPECT_FALSE(isOpponentWithinXMeterFromOwnGoal());
}

TEST(TestSuiteWorldStateFunctionsGetPotentialOpponentAttacker, PotentialOpponentAttacker_test1)
{
	// Setup
	// Update WorldModel administration
	// TESTCASE: Bot1 @ 1,-6, Bot2 @ -1,-7, Ball @ -1.1,-7.0  => Bot2 closest to goal but also to ball

	// set ball location
	ballLocation ballLocation;
    ballLocations newBallLocations;
    ballLocation.position.x = -1.1;
    ballLocation.position.y = -7.0;
    newBallLocations.push_back(ballLocation);
	cWorldModelInterface::getInstance().setBallLocation(newBallLocations, ballLocation);

	// set other bot location(s) and stuff
    robotLocations opponents;
    // --R1
    robotLocation opp1;
	opp1.position = geometry::Pose2D(1.0, -6.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(1, opp1));
	// --R2
    robotLocation opp2;
	opp2.position = geometry::Pose2D(-1.0, -7.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(2, opp2));

	cWorldModelInterface::getInstance().setOpponents(opponents);

    // Execution
	float x,y;
	bool retval = cWorldStateFunctions::getInstance().getPotentialOpponentAttacker(x,y);
	bool retval2 = isPotentialOppAttackerPresent();

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
	ballLocation ballLocation;
    ballLocations newBallLocations;
    ballLocation.position.x = 1.1;
    ballLocation.position.y = -6.0;
    newBallLocations.push_back(ballLocation);
	cWorldModelInterface::getInstance().setBallLocation(newBallLocations, ballLocation);

	// set other bot location(s) and stuff
    robotLocations opponents;
    // --R1
    robotLocation opp1;
	opp1.position = geometry::Pose2D(1.0, -6.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(1, opp1));
	// --R2
    robotLocation opp2;
	opp2.position = geometry::Pose2D(-1.0, -7.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(2, opp2));

	cWorldModelInterface::getInstance().setOpponents(opponents);

    // Execution
	float x,y;
	bool retval = cWorldStateFunctions::getInstance().getPotentialOpponentAttacker(x,y);
	bool retval2 = isPotentialOppAttackerPresent();

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
	ballLocation ballLocation;
    ballLocations newBallLocations;
    ballLocation.position.x = 1.1;
    ballLocation.position.y = -6.0;
    newBallLocations.push_back(ballLocation);
	cWorldModelInterface::getInstance().setBallLocation(newBallLocations, ballLocation);

	// set other bot location(s) and stuff
    robotLocations opponents;
    // --R1
    robotLocation opp1;
	opp1.position = geometry::Pose2D(1.0, -6.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(1, opp1));

	cWorldModelInterface::getInstance().setOpponents(opponents);

    // Execution
	float x,y;
	bool retval = cWorldStateFunctions::getInstance().getPotentialOpponentAttacker(x,y);
	bool retval2 = isPotentialOppAttackerPresent();

	// Verification
	EXPECT_TRUE(retval == false);
	EXPECT_TRUE(retval2 == false);
}

TEST(TestSuiteWorldStateFunctionsBallPath, calcBallPath)
{
	/* Setup */
	// set ball
	ballLocation ourBall;
	ourBall.position.x=5.0;
	ourBall.position.y=-4.0;
	ourBall.velocity.x=-5.0;
	ourBall.velocity.y=-5.0;

	// yIntercept = -9.05  , slope = 1

	cObjectPath ballLine( ourBall.position, ourBall.velocity );

	/* Verification */
	// y = YIntercept + Slope * x
	EXPECT_FLOAT_EQ( -9.0f, ballLine.getYIntercept() );
	EXPECT_FLOAT_EQ( 1.0f, ballLine.getSlope() );
	float expectY = ballLine.getYIntercept() +  ballLine.getSlope() * 100.0;
	EXPECT_FLOAT_EQ( expectY, ballLine.calcY(100.0));  //check Y with X=100.0
	// y = 9,  x expected = (9- -9.05)/0.8
	float expectX = (9.0 - ballLine.getYIntercept())/ballLine.getSlope();
	EXPECT_FLOAT_EQ( expectX, ballLine.calcX(9.0)); // check X with Y=9.0
    EXPECT_TRUE( ballLine.intersectCheck( Point2D(-9.0, -9.0), Point2D(9.0, -9.0)) );

    EXPECT_FALSE( ballLine.intersectCheck( Point2D(-3.0, -0.0), Point2D( 3.0, -0.0) ));


    expectY=-9.0;
    expectX=(expectY - ballLine.getYIntercept())/ballLine.getSlope();
    Point2D intersectCoord;
    bool b= ballLine.intersectCheck( Point2D(-9.0, -9.0), Point2D(9.0, -9.0), intersectCoord);
    EXPECT_TRUE( b );
    EXPECT_NEAR( expectX, intersectCoord.x, 0.000001);
    EXPECT_NEAR( expectY, intersectCoord.y, 0.000001);

}

TEST(areTurnsToGoalBlockedByOpponent, Test1)
{
    Position2D ownLocation(-4.0, 2.0, M_PI);
    cWorldModelInterface::getInstance().setOwnLocation( ownLocation );

    robotLocations opponents;
    robotLocation opponent;
    opponent.position = geometry::Pose2D(-4.0, 0.9, 0.0);
    opponents.insert(std::pair<robotNumber, robotLocation>(1, opponent));
    opponent.position = geometry::Pose2D(-3.0, 3.0, 0.0);
    opponents.insert(std::pair<robotNumber, robotLocation>(2, opponent));
    opponent.position = geometry::Pose2D(-1.0, 7.0, 0.0);
    opponents.insert(std::pair<robotNumber, robotLocation>(3, opponent));
    cWorldModelInterface::getInstance().setOpponents(opponents);

    EXPECT_FALSE(isShortTurnToGoalBlockedByOpponent());
    EXPECT_FALSE(isLongTurnToGoalBlockedByOpponent());
}

TEST(areTurnsToGoalBlockedByOpponent, Test2)
{
    Position2D ownLocation(-4.0, 2.0, M_PI);
    cWorldModelInterface::getInstance().setOwnLocation( ownLocation );

    robotLocations opponents;
    robotLocation opponent;
    opponent.position = geometry::Pose2D(-4.0, 0.9, 0.0);
    opponents.insert(std::pair<robotNumber, robotLocation>(1, opponent));
    opponent.position = geometry::Pose2D(-4.0, 2.9, 0.0);
    opponents.insert(std::pair<robotNumber, robotLocation>(2, opponent));
    opponent.position = geometry::Pose2D(-1.0, 7.0, 0.0);
    opponents.insert(std::pair<robotNumber, robotLocation>(3, opponent));
    cWorldModelInterface::getInstance().setOpponents(opponents);

    EXPECT_TRUE(isShortTurnToGoalBlockedByOpponent());
    EXPECT_FALSE(isLongTurnToGoalBlockedByOpponent());
}

TEST(areTurnsToGoalBlockedByOpponent, Test3)
{
    Position2D ownLocation(-4.0, 2.0, 0.0);
    cWorldModelInterface::getInstance().setOwnLocation( ownLocation );

    robotLocations opponents;
    robotLocation opponent;
    opponent.position = geometry::Pose2D(-4.0, 0.9, 0.0);
    opponents.insert(std::pair<robotNumber, robotLocation>(1, opponent));
    opponent.position = geometry::Pose2D(-4.0, 2.9, 0.0);
    opponents.insert(std::pair<robotNumber, robotLocation>(2, opponent));
    opponent.position = geometry::Pose2D(-1.0, 7.0, 0.0);
    opponents.insert(std::pair<robotNumber, robotLocation>(3, opponent));
    cWorldModelInterface::getInstance().setOpponents(opponents);

    EXPECT_FALSE(isShortTurnToGoalBlockedByOpponent());
    EXPECT_TRUE(isLongTurnToGoalBlockedByOpponent());
}

TEST(areTurnsToGoalBlockedByOpponent, Test4)
{
    Position2D ownLocation(-4.0, 2.0, M_PI);
    cWorldModelInterface::getInstance().setOwnLocation( ownLocation );

    robotLocations opponents;
    robotLocation opponent;
    opponent.position = geometry::Pose2D(-4.0, 1.1, 0.0);
    opponents.insert(std::pair<robotNumber, robotLocation>(1, opponent));
    opponent.position = geometry::Pose2D(-4.0, 2.9, 0.0);
    opponents.insert(std::pair<robotNumber, robotLocation>(2, opponent));
    opponent.position = geometry::Pose2D(-1.0, 7.0, 0.0);
    opponents.insert(std::pair<robotNumber, robotLocation>(3, opponent));
    cWorldModelInterface::getInstance().setOpponents(opponents);

    EXPECT_TRUE(isShortTurnToGoalBlockedByOpponent());
    EXPECT_TRUE(isLongTurnToGoalBlockedByOpponent());
}

TEST(areTurnsToGoalBlockedByOpponent, Test5)
{
    Position2D ownLocation(0.0, 5.0, (1.4 * M_PI));
    cWorldModelInterface::getInstance().setOwnLocation( ownLocation );

    robotLocations opponents;
    robotLocation opponent;
    opponent.position = geometry::Pose2D(-0.5, 7.0, 0.0);
    opponents.insert(std::pair<robotNumber, robotLocation>(1, opponent));
    opponent.position = geometry::Pose2D(0.5, 7.0, 0.0);
    opponents.insert(std::pair<robotNumber, robotLocation>(2, opponent));
    cWorldModelInterface::getInstance().setOpponents(opponents);

    EXPECT_FALSE(isShortTurnToGoalBlockedByOpponent());
    EXPECT_FALSE(isLongTurnToGoalBlockedByOpponent());
}

TEST(areTurnsToGoalBlockedByOpponent, Test6)
{
    Position2D ownLocation(0.0, 5.0, (1.4 * M_PI));
    cWorldModelInterface::getInstance().setOwnLocation( ownLocation );

    robotLocations opponents;
    robotLocation opponent;
    opponent.position = geometry::Pose2D(-0.5, 5.5, 0.0);
    opponents.insert(std::pair<robotNumber, robotLocation>(1, opponent));
    opponent.position = geometry::Pose2D(0.5, 7.0, 0.0);
    opponents.insert(std::pair<robotNumber, robotLocation>(2, opponent));
    cWorldModelInterface::getInstance().setOpponents(opponents);

    EXPECT_TRUE(isShortTurnToGoalBlockedByOpponent());
    EXPECT_FALSE(isLongTurnToGoalBlockedByOpponent());
}

TEST(areTurnsToGoalBlockedByOpponent, Test7)
{
    Position2D ownLocation(0.0, 5.0, (1.4 * M_PI));
    cWorldModelInterface::getInstance().setOwnLocation( ownLocation );

    robotLocations opponents;
    robotLocation opponent;
    opponent.position = geometry::Pose2D(-0.5, 7.0, 0.0);
    opponents.insert(std::pair<robotNumber, robotLocation>(1, opponent));
    opponent.position = geometry::Pose2D(0.5, 5.5, 0.0);
    opponents.insert(std::pair<robotNumber, robotLocation>(2, opponent));
    cWorldModelInterface::getInstance().setOpponents(opponents);

    EXPECT_FALSE(isShortTurnToGoalBlockedByOpponent());
    EXPECT_TRUE(isLongTurnToGoalBlockedByOpponent());
}

TEST(areTurnsToGoalBlockedByOpponent, Test8)
{
    Position2D ownLocation(3.0, 7.0, (2.0 * M_PI));
    cWorldModelInterface::getInstance().setOwnLocation( ownLocation );

    robotLocations opponents;
    robotLocation opponent;
    opponent.position = geometry::Pose2D(2.5, 7.5, 0.0);
    opponents.insert(std::pair<robotNumber, robotLocation>(1, opponent));
    cWorldModelInterface::getInstance().setOpponents(opponents);

    EXPECT_TRUE(isShortTurnToGoalBlockedByOpponent());
    EXPECT_FALSE(isLongTurnToGoalBlockedByOpponent());
}

TEST(areTurnsToGoalBlockedByOpponent, Test9)
{
    Position2D ownLocation(3.0, 7.0, (1.5 * M_PI));
    cWorldModelInterface::getInstance().setOwnLocation( ownLocation );

    robotLocations opponents;
    robotLocation opponent;
    opponent.position = geometry::Pose2D(2.5, 7.0, 0.0);
    opponents.insert(std::pair<robotNumber, robotLocation>(1, opponent));
    cWorldModelInterface::getInstance().setOpponents(opponents);

    EXPECT_TRUE(isShortTurnToGoalBlockedByOpponent());
    EXPECT_FALSE(isLongTurnToGoalBlockedByOpponent());
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
	teamplay::configurationStore::getConfiguration().setShootPathWidth(0.25);

	// set other bot location(s) and stuff
    robotLocations opponents;
    // --R8
    robotLocation opp1;
	opp1.position = geometry::Pose2D(0.0, 6.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(1, opp1));
	// --R15
    robotLocation opp2;
	opp2.position = geometry::Pose2D(0.0, 4.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(2, opp2));

	cWorldModelInterface::getInstance().setOpponents(opponents);

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
	teamplay::configurationStore::getConfiguration().setShootPathWidth(0.25);

	// set other bot location(s) and stuff
    robotLocations opponents;
    // --R8
    robotLocation opp1;
	opp1.position = geometry::Pose2D(0.0, 6.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(1, opp1));
	// --R15
    robotLocation opp2;
	opp2.position = geometry::Pose2D(0.0, 5.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(2, opp2));

	cWorldModelInterface::getInstance().setOpponents(opponents);

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
	teamplay::configurationStore::getConfiguration().setShootPathWidth(0.25);

	// set other bot location(s) and stuff
    robotLocations opponents;
    // --R8
    robotLocation opp1;
	opp1.position = geometry::Pose2D(0.0, 6.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(1, opp1));
	// --R15
    robotLocation opp2;
	opp2.position = geometry::Pose2D(0.0, 4.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(2, opp2));

	cWorldModelInterface::getInstance().setOpponents(opponents);

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
	// Member2 @0.5
	// Target @0,5
	// Radius ball = 0.125

	double radiusBall = 0.125;
	std::vector<robotLocation> obstacles;
	teamplay::configurationStore::getConfiguration().setShootPathWidth(0.25);

	// Set teammember on Target
    std::vector<robotNumber> activeRobots;
    robotLocations teammembers;

    activeRobots.push_back(2);
    robotLocation mate2;
    mate2.position = geometry::Pose2D(0.0, 5.0, 0.0);
    teammembers.insert(std::pair<robotNumber, robotLocation>(2, mate2));
    cWorldModelInterface::getInstance().setTeammembers(teammembers, activeRobots);

	// set other bot location(s) and stuff
    robotLocations opponents;
    // --R1
    robotLocation opp1;
	opp1.position = geometry::Pose2D(0.0, 6.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(1, opp1));
	cWorldModelInterface::getInstance().setOpponents(opponents);

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
	// Member2 @0.5
	// Bot1 @0,2
	// Radius ball = 0.125

	double radiusBall = 0.125;
	std::vector<robotLocation> obstacles;
	teamplay::configurationStore::getConfiguration().setShootPathWidth(0.25);

    Position2D ownLocation(0.0, 0.0, 0.0);
    cWorldModelInterface::getInstance().setOwnLocation( ownLocation );

	// Set teammember on Target
    std::vector<robotNumber> activeRobots;
    robotLocations teammembers;

    activeRobots.push_back(2);
    robotLocation mate2;
    mate2.position = geometry::Pose2D(0.0, 5.0, 0.0);
    teammembers.insert(std::pair<robotNumber, robotLocation>(2, mate2));
    cWorldModelInterface::getInstance().setTeammembers(teammembers, activeRobots);

	// set other bot location(s) and stuff
    robotLocations opponents;
    // --R1
    robotLocation opp1;
	opp1.position = geometry::Pose2D(0.0, 2.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(1, opp1));
	cWorldModelInterface::getInstance().setOpponents(opponents);

    // Execution
    EXPECT_TRUE(isPassToClosestTeammemberBlocked());
}

TEST(TestSuiteWorldStateFunctionsisPassToClosestTeammemberBlocked, Teammember2)
{
	// Setup
	// Update WorldModel administration
	// TESTCASE: OWN @0,0,
	// Member2 @0.5
	// Bot1 @0,7
	// Radius ball = 0.125

	double radiusBall = 0.125;
	std::vector<robotLocation> obstacles;
	teamplay::configurationStore::getConfiguration().setShootPathWidth(0.25);

    Position2D ownLocation(0.0, 0.0, 0.0);
    cWorldModelInterface::getInstance().setOwnLocation( ownLocation );

	// Set teammember on Target
    std::vector<robotNumber> activeRobots;
    robotLocations teammembers;

    activeRobots.push_back(2);
    robotLocation mate2;
    mate2.position = geometry::Pose2D(0.0, 5.0, 0.0);
    teammembers.insert(std::pair<robotNumber, robotLocation>(2, mate2));
    cWorldModelInterface::getInstance().setTeammembers(teammembers, activeRobots);

	// set other bot location(s) and stuff
    robotLocations opponents;
    // --R1
    robotLocation opp1;
	opp1.position = geometry::Pose2D(0.0, 7.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(1, opp1));
	cWorldModelInterface::getInstance().setOpponents(opponents);

    // Execution
    EXPECT_FALSE(isPassToClosestTeammemberBlocked());
}

TEST(TestSuiteWorldStateFunctionsisLobShotOnGoalBlocked, Opponent1)
{
	// Setup
	// Update WorldModel administration
	// TESTCASE: OWN @0,0,
	// Bot1 @0,1
	// Radius ball = 0.125

	double radiusBall = 0.125;
	std::vector<robotLocation> obstacles;
	teamplay::configurationStore::getConfiguration().setShootPathWidth(0.25);

    Position2D ownLocation(0.0, 0.0, 0.0);
    cWorldModelInterface::getInstance().setOwnLocation( ownLocation );

	// Set teammember on Target
    std::vector<robotNumber> activeRobots;
    robotLocations teammembers;

	// set other bot location(s) and stuff
    robotLocations opponents;
    // --R1
    robotLocation opp1;
	opp1.position = geometry::Pose2D(0.0, 1.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(1, opp1));
	cWorldModelInterface::getInstance().setOpponents(opponents);

    // Execution
    EXPECT_FALSE(isLobShotOnGoalBlocked());
}

TEST(TestSuiteWorldStateFunctionsisPathToBallBlocked, PathBlocked_test1)
{
	// Setup
	// Update WorldModel administration
	// TESTCASE: OWN @0,0,
	// Bal @0,6
	// Bot1 @0,4 --> robot is obstructing path ball<>robot
	// Radius ball = 0.125

	double radiusBall = 0.125;
	std::vector<robotLocation> obstacles;
	teamplay::configurationStore::getConfiguration().setShootPathWidth(0.25);

    Position2D ownLocation(0.0, 0.0, 0.0);
    cWorldModelInterface::getInstance().setOwnLocation( ownLocation );

	// set ball location
	ballLocation ballLocation;
    ballLocations newBallLocations;
    ballLocation.position.x = 0.0;
    ballLocation.position.y = 6.0;
    newBallLocations.push_back(ballLocation);
	cWorldModelInterface::getInstance().setBallLocation(newBallLocations, ballLocation);

	// Clear teammembers
	std::vector<robotNumber> activeRobots;
	robotLocations teammembers;
	cWorldModelInterface::getInstance().setTeammembers(teammembers, activeRobots);

	// set other bot location(s) and stuff
    robotLocations opponents;
    // --R1
    robotLocation opp1;
	opp1.position = geometry::Pose2D(0.0, 4.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(1, opp1));
	cWorldModelInterface::getInstance().setOpponents(opponents);

    // Execution
    EXPECT_TRUE(isPathToBallBlocked());
}

TEST(TestSuiteWorldStateFunctionsisPathToBallBlocked, PathBlocked_test2)
{
	// Setup
	// Update WorldModel administration
	// TESTCASE: OWN @0,0,
	// Bal @0,6
	// Bot1 @3,3 --> robot is NOT obstructing path ball<>robot
	// Radius ball = 0.125

	double radiusBall = 0.125;
	std::vector<robotLocation> obstacles;
	teamplay::configurationStore::getConfiguration().setShootPathWidth(0.25);

    Position2D ownLocation(0.0, 0.0, 0.0);
    cWorldModelInterface::getInstance().setOwnLocation( ownLocation );

	// set ball location
	ballLocation ballLocation;
    ballLocations newBallLocations;
    ballLocation.position.x = 0.0;
    ballLocation.position.y = 6.0;
    newBallLocations.push_back(ballLocation);
	cWorldModelInterface::getInstance().setBallLocation(newBallLocations, ballLocation);

	// Clear teammembers
	std::vector<robotNumber> activeRobots;
	robotLocations teammembers;
	cWorldModelInterface::getInstance().setTeammembers(teammembers, activeRobots);

	// set other bot location(s) and stuff
    robotLocations opponents;
    // --R1
    robotLocation opp1;
	opp1.position = geometry::Pose2D(3.0, 3.0, 0.0);
	opponents.insert(std::pair<robotNumber, robotLocation>(1, opp1));
	cWorldModelInterface::getInstance().setOpponents(opponents);

    // Execution
    EXPECT_FALSE(isPathToBallBlocked());
}

TEST(TestSuiteWorldStateFunctionsDefendingStrategyOnTest, defendingStrategyTest)
{
	// Setup
	// set setDefendingStrategy to true

	teamplay::configurationStore::getConfiguration().setDefendingStrategy(true);
	bool retval = defendingStrategyOn();

	// Verification
	EXPECT_TRUE(retval == true);
}

TEST(TestSuiteWorldStateFunctionsDefendingStrategyOnTest, defendingStrategyTest2)
{
	// Setup
	// set setDefendingStrategy to true

	teamplay::configurationStore::getConfiguration().setDefendingStrategy(false);
	bool retval = defendingStrategyOn();

	// Verification
	EXPECT_TRUE(retval == false);
}

TEST(TestSuiteWorldStateFunctionsisOpponentHalfReachable, OpponentHalfReachable_test1)
{
	// Setup
	// Update WorldModel administration
	// TESTCASE: OWN @0,-0.1,
	// Ball same as own.picked up at (0, 0.1)

    Position2D ownLocation(0.0, -0.1, 0.0);
    cWorldModelInterface::getInstance().setOwnLocation( ownLocation );

	// set ball location
	ballLocation ballLocation;
    ballLocations newBallLocations;
    ballLocation.position.x = 0.0;
    ballLocation.position.y = 0.1;
    ballLocation.position.z = 0.0;
    newBallLocations.push_back(ballLocation);
	cWorldModelInterface::getInstance().setBallLocation(newBallLocations, ballLocation);
	cWorldModelInterface::getInstance().setBallClaimedLocation(ballLocation.position);

    // Execution
    EXPECT_TRUE(isOpponentHalfReachable());
}

TEST(TestSuiteWorldStateFunctionsisOpponentHalfReachable, OpponentHalfReachable_test2)
{
	// Setup
	// Update WorldModel administration
	// TESTCASE: OWN @0,-1.5,
	// Ball same as own.picked up at (0, -1.5)

    Position2D ownLocation(0.0, -1.5, 0.0);
    cWorldModelInterface::getInstance().setOwnLocation( ownLocation );

	// set ball location
	ballLocation ballLocation;
    ballLocations newBallLocations;
    ballLocation.position.x = 0.0;
    ballLocation.position.y = 1.5;
    ballLocation.position.z = 0.0;
    newBallLocations.push_back(ballLocation);
	cWorldModelInterface::getInstance().setBallLocation(newBallLocations, ballLocation);
	cWorldModelInterface::getInstance().setBallClaimedLocation(ballLocation.position);

    // Execution
    EXPECT_TRUE(isOpponentHalfReachable());
}

TEST(TestSuiteWorldStateFunctionsClosestAttackerToOppGoal, closestAttToOppGoal_test1)
{
	// Setup
	// Update WorldModel administration
	// TESTCASE: OWN @0,0,
	// Member1 @0,5
	// Member2 @0,7

    Position2D ownLocation(0.0, 0.0, 0.0);
    cWorldModelInterface::getInstance().setOwnLocation( ownLocation );

	// Clear teammembers
	std::vector<robotNumber> activeRobots;
	robotLocations teammembers;
	cWorldModelInterface::getInstance().setTeammembers(teammembers, activeRobots);

    activeRobots.push_back(1);
    robotLocation mate1;
    mate1.position = geometry::Pose2D(0.0, 5.0, 0.0);
    teammembers.insert(std::pair<robotNumber, robotLocation>(1, mate1));

    activeRobots.push_back(2);
	robotLocation mate2;
	mate2.position = geometry::Pose2D(0.0, 7.0, 0.0);
	teammembers.insert(std::pair<robotNumber, robotLocation>(2, mate2));

    cWorldModelInterface::getInstance().setTeammembers(teammembers, activeRobots);

    cWorldStateFunctions::getInstance().setRobotRole(1, treeEnum::ATTACKER_MAIN);
    cWorldStateFunctions::getInstance().setRobotRole(2, treeEnum::ATTACKER_ASSIST);

    // Execution
	double x,y;
	cWorldStateFunctions::getInstance().getClosestAttackerToOpponentGoal(x,y);

	// Verification
	EXPECT_FLOAT_EQ( 0.0f, x);
	EXPECT_FLOAT_EQ( 7.0f, y);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  //Enable tracing
  teamplay::traceRedirect::getInstance().setAllTracesToStdout();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
