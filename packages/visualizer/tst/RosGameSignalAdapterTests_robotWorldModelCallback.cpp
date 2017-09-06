 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Integration tests for RosGameSignalAdapter. Require roscore to be running.
 * RosGameSignalAdapterTests.cpp
 *
 *  Created on: May 8, 2016
 *      Author: Diana Koenraadt
 */
#include <string>
#include <math.h>
#include <time.h>

#include <ros/serialization.h>
#include <ros/ros.h>

#include <gtest/gtest.h>

#include "int/RosGameSignalAdapter.h"
#include "GameSignalSubscriberStub.h"

// Falcons shared code:
#include "rosMsgs/t_object.h"

/* 
* ========================================
*           robotWorldModelCallback
* ======================================== 
*/

// Parametrization
class RobotTest : public ::testing::TestWithParam<unsigned int>
{
};

TEST_P(RobotTest, robotWorldModelCallback_onBallPositionChangedCalled)
{
    // Arrange
    std::string nodename = "visualizer";
    int argc = 0;
    ros::init(argc, NULL, nodename);

    ros::NodeHandle n;

    // Retrieve test case specific data
    unsigned int id = GetParam();
    boost::format format_topic("/teamA/robot%1%/g_diag_worldmodel");
    std::string topicN = boost::str(format_topic % id);

    ros::Publisher pub = n.advertise<rosMsgs::t_diag_worldmodel>(topicN, 10);

    RosGameSignalAdapter adapter;
    GameSignalSubscriberStub subscriber(&adapter);
    subscriber.subscribeBallPositionChanged();

    // set up message for publishing
    rosMsgs::t_diag_worldmodel msg;
    msg.ballPresent = true;
    rosMsgs::t_ball ball;
    ball.x = 10;
    ball.y = 10;
    msg.ballpos.push_back(ball);

    // Act 
    pub.publish(msg);

    // Assert
    while (subscriber.receivedMessages.size() == 0)
    {
        // Required for message transfer. Note that test has timeout parameter in CMakeLists, ensuring termination.
        ros::spinOnce();
    }

    ASSERT_EQ(1, subscriber.receivedMessages.size());

    // Set up test case specific results
    boost::format format_result("onBallPositionChanged { id : %1%, x : 10.00, y : 10.00, z : 0.00, vx : 0.00, vy : 0.00, vz : 0.00 }");
    ASSERT_EQ(boost::str(format_result % id), subscriber.receivedMessages[0]);
}

INSTANTIATE_TEST_CASE_P(robotWorldModelCallback_onBallPositionChanged, RobotTest, testing::Values(1, 2, 3, 4, 5, 6));

TEST_P(RobotTest, robotWorldModelCallback_onBallPossessionChangedCalled)
{
    // Arrange
    std::string nodename = "visualizer";
    int argc = 0;
    ros::init(argc, NULL, nodename);

    ros::NodeHandle n;

    // Retrieve test case specific data
    unsigned int id = GetParam();
    boost::format format_topic("/teamA/robot%1%/g_diag_worldmodel");
    std::string topicN = boost::str(format_topic % id);

    ros::Publisher pub = n.advertise<rosMsgs::t_diag_worldmodel>(topicN, 10);

    RosGameSignalAdapter adapter;
    GameSignalSubscriberStub subscriber(&adapter);
    subscriber.subscribeBallPossessionChanged();

    // set up message for publishing
    rosMsgs::t_diag_worldmodel msg;
    msg.ballPossession.type = TYPE_OPPONENT;
    msg.ballPossession.robotID = 2;

    // Act 
    pub.publish(msg);

    // Assert
    while (subscriber.receivedMessages.size() == 0)
    {
        // Required for message transfer. Note that test has timeout parameter in CMakeLists, ensuring termination.
        ros::spinOnce();
    }

    ASSERT_EQ(1, subscriber.receivedMessages.size());

    // Set up test case specific results
    boost::format format_result("onBallPossessionChanged { senderRobotId : %1%, ballPossessionType : 2, robotId : 2 }");
    ASSERT_EQ(boost::str(format_result % id), subscriber.receivedMessages[0]);
}

INSTANTIATE_TEST_CASE_P(robotWorldModelCallback_onBallPossessionChanged, RobotTest, testing::Values(1, 2, 3, 4, 5, 6));

TEST_P(RobotTest, robotWorldModelCallback_onObstaclePositionChangedCalled)
{
    // Arrange
    std::string nodename = "visualizer";
    int argc = 0;
    ros::init(argc, NULL, nodename);

    ros::NodeHandle n;

    // Retrieve test case specific data
    unsigned int id = GetParam();
    boost::format format_topic("/teamA/robot%1%/g_diag_worldmodel");
    std::string topicN = boost::str(format_topic % id);

    ros::Publisher pub = n.advertise<rosMsgs::t_diag_worldmodel>(topicN, 10);

    RosGameSignalAdapter adapter;
    GameSignalSubscriberStub subscriber(&adapter);
    subscriber.subscribeObstaclePositionChanged();

    // set up message for publishing
    rosMsgs::t_diag_worldmodel msg;
    rosMsgs::t_object enemy;
    enemy.id = 1;
    enemy.x = 20;
    enemy.y = 22;
    enemy.phi = 1.34;
    enemy.vx = 1;
    enemy.vy = 0.4;
    enemy.vphi = 1.44;
    msg.enemies.push_back(enemy);

    // Act 
    pub.publish(msg);

    // Assert
    while (subscriber.receivedMessages.size() == 0)
    {
        // Required for message transfer. Note that test has timeout parameter in CMakeLists, ensuring termination.
        ros::spinOnce();
    }

    ASSERT_EQ(1, subscriber.receivedMessages.size());

    // Set up test case specific results
    boost::format format_result("onObstaclePositionChanged { id : %1%, x : 20.00, y : 22.00, phi : 1.34, vx : 1.00, vy : 0.40, vphi : 1.44 }");
    ASSERT_EQ(boost::str(format_result % id), subscriber.receivedMessages[0]);
}

INSTANTIATE_TEST_CASE_P(robotWorldModelCallback_onObstaclePositionChanged, RobotTest, testing::Values(1, 2, 3, 4, 5, 6));

TEST_P(RobotTest, robotWorldModelCallback_onOwnTeamPositionChangedCalled)
{
    // Arrange
    std::string nodename = "visualizer";
    int argc = 0;
    ros::init(argc, NULL, nodename);

    ros::NodeHandle n;

    // Retrieve test case specific data
    unsigned int id = GetParam();
    boost::format format_topic("/teamA/robot%1%/g_diag_worldmodel");
    std::string topicN = boost::str(format_topic % id);

    ros::Publisher pub = n.advertise<rosMsgs::t_diag_worldmodel>(topicN, 10);

    RosGameSignalAdapter adapter;
    GameSignalSubscriberStub subscriber(&adapter);
    subscriber.subscribeOwnTeamPositionChanged();

    // set up message for publishing
    rosMsgs::t_diag_worldmodel msg;
    msg.ownpos.x = 21;
    msg.ownpos.y = 23;
    msg.ownpos.phi = 1.35;
    msg.ownpos.vx = 2;
    msg.ownpos.vy = 0.5;

    rosMsgs::t_object tfriend;
    tfriend.id = 1;
    tfriend.x = 20;
    tfriend.y = 22;
    tfriend.phi = 1.34;
    tfriend.vx = 1;
    tfriend.vy = 0.4;
    tfriend.vphi = 1.44;
    msg.friends.push_back(tfriend);

    // Act 
    pub.publish(msg);

    // Assert
    while (subscriber.receivedMessages.size() == 0)
    {
        // Required for message transfer. Note that test has timeout parameter in CMakeLists, ensuring termination.
        ros::spinOnce();
    }

    ASSERT_EQ(2, subscriber.receivedMessages.size());

    // Set up test case specific results
    boost::format format_result1("onOwnTeamPositionChanged { senderRobotId : %1%, robotId : 1, x : 20.00, y : 22.00, phi : 1.34, vx : 1.00, vy : 0.40, vphi : 1.44 }");
    ASSERT_EQ(boost::str(format_result1 % id), subscriber.receivedMessages[0]);
    boost::format format_result2("onOwnTeamPositionChanged { senderRobotId : %1%, robotId : %1%, x : 21.00, y : 23.00, phi : 1.35, vx : 2.00, vy : 0.50, vphi : 0.00 }");
    ASSERT_EQ(boost::str(format_result2 % id), subscriber.receivedMessages[1]);
}

INSTANTIATE_TEST_CASE_P(robotWorldModelCallback_onOwnTeamPositionChanged, RobotTest, testing::Values(1, 2, 3, 4, 5, 6));

/*
 * Main entry
 */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
