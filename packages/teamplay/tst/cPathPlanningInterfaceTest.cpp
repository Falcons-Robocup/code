 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPathPlanningInterfaceTest.cpp
 * 
 * A roscore needs to be active for running this test.
 * Test that the ROS input adapter can read services and relays the data to PathPlanning.
 * Services are mocked by cPathPlanningRosStub
 *
 *  Created on: Dec 29, 2015
 *      Author: Coen Tempelaars
 */

#include "gtest/gtest.h"
#include "int/utilities/trace.hpp"
#include "ros/ros.h"

#include <boost/thread/thread.hpp>

#include "cPathPlanningRosStub.hpp"

// system under test
#include "int/adapters/cPathPlanningInterface.hpp"


// test utilities
cPathPlanningRosStub *pathPlanningRosStub;

static void spinThread()
{
    ros::spin();
}


TEST(PathPlanningInterface, TestGetSetActive)
{
    // setup
    cPathPlanningInterface pathPlanningInterface;
    pathPlanningInterface.connect();
    pathPlanningRosStub->reset();
    EXPECT_FALSE(pathPlanningInterface.isEnabled());

    // execution
    pathPlanningInterface.enable();
    EXPECT_TRUE(pathPlanningInterface.isEnabled());

    pathPlanningInterface.enable();
    EXPECT_TRUE(pathPlanningInterface.isEnabled());

    pathPlanningInterface.disable();
    EXPECT_FALSE(pathPlanningInterface.isEnabled());

    pathPlanningInterface.disable();
    EXPECT_FALSE(pathPlanningInterface.isEnabled());

    pathPlanningInterface.enable();
    EXPECT_TRUE(pathPlanningInterface.isEnabled());

    // teardown
    pathPlanningRosStub->reset();
}


TEST(PathPlanningInterface, TestMove)
{
    // setup
    cPathPlanningInterface pathPlanningInterface;
    pathPlanningInterface.connect();
    pathPlanningRosStub->reset();
    EXPECT_NEAR(0.000, pathPlanningRosStub->getPose().getX(), 0.0001);
    EXPECT_NEAR(0.000, pathPlanningRosStub->getPose().getY(), 0.0001);
    EXPECT_NEAR(0.000, pathPlanningRosStub->getPose().getPhi(), 0.0001);

    // execution
    geometry::Pose2D dest (1.0, 2.0, 0.5);
    std::String motionProfile = "normal";
    pathPlanningInterface.moveTo(dest, motionProfile);
    sleep(1);

    // assertions/expectations
    EXPECT_NEAR(1.000, pathPlanningRosStub->getPose().getX(), 0.0001);
    EXPECT_NEAR(2.000, pathPlanningRosStub->getPose().getY(), 0.0001);
    EXPECT_NEAR(0.500, pathPlanningRosStub->getPose().getPhi(), 0.0001);

    // execution
    dest.teleport(2.0, 4.0, 1.0);
    pathPlanningInterface.moveTo(dest, motionProfile);
    sleep(1);

    // assertions/expectations
    EXPECT_NEAR(2.000, pathPlanningRosStub->getPose().getX(), 0.0001);
    EXPECT_NEAR(4.000, pathPlanningRosStub->getPose().getY(), 0.0001);
    EXPECT_NEAR(1.000, pathPlanningRosStub->getPose().getPhi(), 0.0001);

    // teardown
    pathPlanningRosStub->reset();
}


int main(int argc, char **argv)
{
    //Enable tracing
    //teamplay::traceRedirect::getInstance().setAllTracesToStdout();

    ros::init(argc, argv, "pathPlanningInterfaceTest");

    // instantiate cWorldModelRosStub and start a ROS spinner thread to prevent deadlock within this process
    pathPlanningRosStub = new cPathPlanningRosStub();
    boost::thread st(&spinThread);

    // Run all the tests that were declared with TEST()
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

