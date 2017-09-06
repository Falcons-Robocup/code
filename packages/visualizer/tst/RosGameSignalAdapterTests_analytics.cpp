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
*           analytics
* ======================================== 
*/

TEST(AnalyticsTest, teamAnalyticsMatchStateCallback_onClockTick)
{
    // Arrange
    std::string nodename = "visualizer";
    int argc = 0;
    ros::init(argc, NULL, nodename);

    ros::NodeHandle n;

    // Retrieve test case specific data
    std::string topic = "/teamA/g_ana_matchstate";

    ros::Publisher pub = n.advertise<rosMsgs::t_ana_matchstate>(topic, 10);

    RosGameSignalAdapter adapter;
    GameSignalSubscriberStub subscriber(&adapter);
    subscriber.subscribeTime();

    // set up message for publishing
    rosMsgs::t_ana_matchstate msg;
    msg.startTime = 60;
    msg.currentTime = 120;

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
    ASSERT_EQ("onClockTick { matchStartTime : 00:01, currentTime : 00:02 }", subscriber.receivedMessages[0]);
}

/*
 * Main entry
 */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}