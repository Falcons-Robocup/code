 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cWorldModelInterfaceTest.cpp
 * 
 * A roscore needs to be active for running this test.
 * Test that the ROS input adapter can read services and relays the data to internal WorldModelInterface.
 * Services are mocked.
 *
 *  Created on: Oct 13, 2015
 *      Author: Jan Feitsma
 */


#include "ros/ros.h"
#include "int/utilities/trace.hpp"
#include <gtest/gtest.h>
#include <boost/thread/thread.hpp>

#include "cWorldModelRosStub.hpp"

#include "int/cRosAdapterWorldModel.hpp"
#include "int/cWorldModelInterface.hpp"


// test utilities
cWorldModelRosStub *g_worldModelRosStub;
int g_updateCount = 0;

static void spinThread()
{
    ros::spin();
}

static void updateWorldModel()
{
    cRosAdapterWorldModel::getInstance().update();
    g_updateCount++;
}

static void resetWorldModel()
{
    while ((g_updateCount % 4) != 0)   //cWorldModelRosStub reaches its initial state after every 4 updates
    {
        cRosAdapterWorldModel::getInstance().update();
        g_updateCount++;
    }
}


// testCase: getOwnLocation
TEST(WorldModelInterfaceAdapter, TestOwnLocation)
{
    // setup: none
    // execution: call update twice (which calls the WorldModel services) and store data
    Position2D robotPosition1;
    Position2D robotPosition2;
    updateWorldModel();
    cWorldModelInterface::getInstance().getOwnLocation(robotPosition1);
    updateWorldModel();
    cWorldModelInterface::getInstance().getOwnLocation(robotPosition2);

    // assertions/expectations
    EXPECT_EQ(robotPosition2.x, 1.0 + robotPosition1.x); // check consecutive calls cause the static counter to increase
    EXPECT_EQ(1.0, robotPosition1.y);
    EXPECT_EQ(1.0, robotPosition2.y);
    EXPECT_EQ(2.0, robotPosition1.phi);
    EXPECT_EQ(2.0, robotPosition2.phi);
    // teardown
    resetWorldModel();
}

// testCase: getTeamMembers
TEST(WorldModelInterfaceAdapter, TestTeammembers)
{
    // setup: none
    // execution: call update twice (which calls the WorldModel services) and store data
    robotLocations teamMembers1;
    robotLocations teamMembers2;
    robotLocations::iterator it1;
    robotLocations::iterator it2;
    int teamMemberId1 = 3;
    int teamMemberId2 = 7;

    updateWorldModel();
    cWorldModelInterface::getInstance().getTeammembers(teamMembers1);
    updateWorldModel();
    cWorldModelInterface::getInstance().getTeammembers(teamMembers2);

    robotLocation robot1_v1 = teamMembers1.find(teamMemberId1)->second;
    robotLocation robot1_v2 = teamMembers2.find(teamMemberId1)->second;
    robotLocation robot2_v2 = teamMembers2.find(teamMemberId2)->second;

    // assertions/expectations
    EXPECT_EQ(1, teamMembers1.size());
    EXPECT_EQ(2, teamMembers2.size());
    EXPECT_EQ(robot1_v2.position.getX(), 1.0 + robot1_v1.position.getX()); // check consecutive calls cause the static counter to increase
    EXPECT_EQ(robot2_v2.position.getX(), 1.0 + robot1_v1.position.getX());
    EXPECT_EQ(-1.0, robot1_v1.position.getY());
    EXPECT_NEAR(((2 * M_PI) - 2.0), robot1_v1.position.getPhi(), 0.0001);
    EXPECT_EQ(-3.0, robot1_v1.velocity.getX());
    EXPECT_EQ(-4.0, robot1_v1.velocity.getY());
    EXPECT_EQ(-5.0, robot1_v1.velocity.getPhi());
    EXPECT_EQ(-6.0, robot1_v1.confidence);
    // teardown:
    resetWorldModel();
}

// testCase: getOpponents
TEST(WorldModelInterfaceAdapter, TestOpponents)
{
	// setup: none
	// execution: call update;getOpponents twice. update will set the opponents.
    robotLocations opponents1;
    robotLocations opponents2;
    robotLocations::iterator it1;
    robotLocations::iterator it2;
    int opponentId1 = 0;
    int opponentId2 = 1;

	updateWorldModel();
	cWorldModelInterface::getInstance().getOpponents(opponents1);
	updateWorldModel();
	cWorldModelInterface::getInstance().getOpponents(opponents2);

	robotLocation opponent1_v1 = opponents1.find(opponentId1)->second;
	robotLocation opponent1_v2 = opponents2.find(opponentId1)->second;
	robotLocation opponent2_v2 = opponents2.find(opponentId2)->second;

	// assertions/expectations
	EXPECT_EQ(1, opponents1.size());
	EXPECT_EQ(2, opponents2.size());
	EXPECT_EQ(opponent1_v2.position.getX(), 1.0 + opponent1_v1.position.getX());
	EXPECT_EQ(opponent2_v2.position.getX(), 1.0 + opponent1_v1.position.getX());
	EXPECT_EQ(-2.0, opponent1_v1.position.getY());
	EXPECT_NEAR(((2 * M_PI) - 3.0), opponent1_v1.position.getPhi(), 0.0001);
	EXPECT_EQ(-4.0, opponent1_v1.velocity.getX());
	EXPECT_EQ(-5.0, opponent1_v1.velocity.getY());
	EXPECT_EQ(-6.0, opponent1_v1.velocity.getPhi());
	EXPECT_EQ(-7.0, opponent1_v1.confidence);
	// teardown:
	resetWorldModel();
}

// testCase: getBallPossession
TEST(WorldModelInterfaceAdapter, TestBallPossession)
{
	// setup: none
	// execution: call update;getBallPossession four times. update will set the ball possession.
	ballPossession_struct_t ballPossessionInvalid;
	ballPossession_struct_t ballPossessionField;
	ballPossession_struct_t ballPossessionOpponent;
	ballPossession_struct_t ballPossessionTeammember;
	updateWorldModel();
	cWorldModelInterface::getInstance().getBallPossession(ballPossessionInvalid);
	updateWorldModel();
	cWorldModelInterface::getInstance().getBallPossession(ballPossessionField);
	updateWorldModel();
	cWorldModelInterface::getInstance().getBallPossession(ballPossessionOpponent);
	updateWorldModel();
	cWorldModelInterface::getInstance().getBallPossession(ballPossessionTeammember);

	// assertions/expectations
	EXPECT_EQ(ballPossessionEnum::INVALID,    ballPossessionInvalid.possessionType);
	EXPECT_EQ(ballPossessionEnum::FIELD,      ballPossessionField.possessionType);
	EXPECT_EQ(ballPossessionEnum::OPPONENT,   ballPossessionOpponent.possessionType);
	EXPECT_EQ(ballPossessionEnum::TEAMMEMBER, ballPossessionTeammember.possessionType);
	EXPECT_EQ(255, ballPossessionInvalid.robotID);
	EXPECT_EQ(255, ballPossessionField.robotID);
	EXPECT_EQ(255, ballPossessionOpponent.robotID);
	EXPECT_EQ(42, ballPossessionTeammember.robotID);

	//teardown:
	resetWorldModel();
}

TEST(WorldModelInterfaceAdapter, TestActiveRobots)
{
	// setup: none
	// execution: call update;getActiveRobots twice. update will set the active robots
	std::vector<robotNumber> robots1;
	std::vector<robotNumber> robots2;
	updateWorldModel();
	cWorldModelInterface::getInstance().getActiveRobots(robots1);
	updateWorldModel();
	cWorldModelInterface::getInstance().getActiveRobots(robots2);

	// assertions/expectations
	EXPECT_EQ(1,  robots1.size());
	EXPECT_EQ(42, robots1.at(0));
	EXPECT_EQ(2,  robots2.size());
	EXPECT_EQ(42, robots2.at(0));
	EXPECT_EQ(43, robots2.at(1));
	// teardown:
	resetWorldModel();
}


// MAIN
int main(int argc, char **argv)
{
    //Enable tracing
    teamplay::traceRedirect::getInstance().setAllTracesToStdout();

    // initialize
    ros::init(argc, argv, "testRosInterfaceWMinput");

    // instantiate cWorldModelRosStub and start a ROS spinner thread to prevent deadlock within this process
    g_worldModelRosStub = new cWorldModelRosStub;
    boost::thread st(&spinThread);

    // Run all the tests that were declared with TEST()
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}


