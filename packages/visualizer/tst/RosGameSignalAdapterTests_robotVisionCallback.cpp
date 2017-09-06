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

// Parametrization
class RobotTest : public ::testing::TestWithParam<unsigned int>
{
};

/*
* ========================================
*           robotVisionCallback
* ======================================== 
*/

TEST_P(RobotTest, robotVisionCallback_onValue)
{
    // Arrange
    std::string nodename = "visualizer";
    int argc = 0;
    ros::init(argc, NULL, nodename);

    ros::NodeHandle n;

    // Retrieve test case specific data
    unsigned int id = GetParam();
    boost::format format_topic("/teamA/robot%1%/g_diag_vision");
    std::string topicN = boost::str(format_topic % id);

    ros::Publisher pub = n.advertise<rosMsgs::t_diag_vision>(topicN, 10);

    RosGameSignalAdapter adapter;
    GameSignalSubscriberStub subscriber(&adapter);
    subscriber.subscribeValue();

    // set up message for publishing
    rosMsgs::t_diag_vision msg;
    msg.fps = 14;
    msg.linePoints = 100;
    msg.age = 1;
    msg.lastActive = 20;
    msg.numBalls = 2;
    msg.numObstacles = 9;

    // Act 
    pub.publish(msg);

    // Assert
    while (subscriber.receivedMessages.size() == 0)
    {
        // Required for message transfer. Note that test has timeout parameter in CMakeLists, ensuring termination.
        ros::spinOnce();
    }

    ASSERT_EQ(4, subscriber.receivedMessages.size());

    // Set up test case specific results
    boost::format format_result("onValue { senderRobotId : %1%, category : VISION, key : %2%, value : %3% }");
    ASSERT_EQ(boost::str(format_result % id % "fps" % 14), subscriber.receivedMessages.at(0));
    ASSERT_EQ(boost::str(format_result % id % "linePoints" % 100), subscriber.receivedMessages.at(1));
    ASSERT_EQ(boost::str(format_result % id % "numBalls" % 2), subscriber.receivedMessages.at(2));
    ASSERT_EQ(boost::str(format_result % id % "numObstacles" % 9), subscriber.receivedMessages.at(3));
}

INSTANTIATE_TEST_CASE_P(robotVisionCallback_onVisionMetaDataChangedCalled, RobotTest, testing::Values(1, 2, 3, 4, 5, 6));

TEST(RobotTest, robotVisionCallback_onBallPositionChanged_Angle0Radius0_ReturnsCenterCoordinates)
{
    // Arrange
    std::string nodename = "visualizer";
    int argc = 0;
    ros::init(argc, NULL, nodename);

    ros::NodeHandle n;

    // Retrieve test case specific data
    unsigned int id = 1;
    boost::format format_topic("/teamA/robot%1%/g_diag_vision");
    std::string topicN = boost::str(format_topic % id);

    ros::Publisher pub = n.advertise<rosMsgs::t_diag_vision>(topicN, 10);

    RosGameSignalAdapter adapter;
    GameSignalSubscriberStub subscriber(&adapter);
    subscriber.subscribeBallPositionChanged();

    // set up message for publishing
    rosMsgs::t_diag_vision msg;
    msg.ownpos.x = 0;
    msg.ownpos.y = 0;
    msg.ownpos.phi = 0;

    rosMsgs::angradconf arc;
    arc.angle = 0; 
    arc.radius = 0;
    msg.ballpos.push_back(arc);

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
    boost::format format_result("onBallPositionChanged { id : %1%, x : 0.00, y : 0.00, z : 0.00, vx : 0.00, vy : 0.00, vz : 0.00 }");
    ASSERT_EQ(boost::str(format_result % id), subscriber.receivedMessages[0]);
}

TEST_P(RobotTest, robotVisionCallback_onBallPositionChanged)
{
    // Arrange
    std::string nodename = "visualizer";
    int argc = 0;
    ros::init(argc, NULL, nodename);

    ros::NodeHandle n;

    // Retrieve test case specific data
    unsigned int id = GetParam();
    boost::format format_topic("/teamA/robot%1%/g_diag_vision");
    std::string topicN = boost::str(format_topic % id);

    ros::Publisher pub = n.advertise<rosMsgs::t_diag_vision>(topicN, 10);

    RosGameSignalAdapter adapter;
    GameSignalSubscriberStub subscriber(&adapter);
    subscriber.subscribeBallPositionChanged();

    // set up message for publishing
    rosMsgs::t_diag_vision msg;
    msg.ownpos.x = 0;
    msg.ownpos.y = 0;
    msg.ownpos.phi = M_PI / 2; // radians, CCW, angle 0 == FCS x+

    rosMsgs::angradconf arc;
    arc.angle = M_PI / 2; // radians, CCW
    arc.radius = 5;
    msg.ballpos.push_back(arc);

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
    boost::format format_result("onBallPositionChanged { id : %1%, x : -0.00, y : 5.00, z : 0.00, vx : 0.00, vy : 0.00, vz : 0.00 }");
    ASSERT_EQ(boost::str(format_result % id), subscriber.receivedMessages[0]);
}

INSTANTIATE_TEST_CASE_P(robotVisionCallback_onBallPositionChanged, RobotTest, testing::Values(1, 2, 3, 4, 5, 6));

/*
 * Main entry
 */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
