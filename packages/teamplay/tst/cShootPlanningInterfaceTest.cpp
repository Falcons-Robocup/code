 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cShootPlanningInterfaceTest.cpp
 *
 * A roscore needs to be active for running this test.
 * Test that the ROS input adapter can read services and relays the data to ShootPlanning.
 * Services are mocked by cShootPlanningRosStub
 *
 *  Created on: Jan 12, 2016
 *      Author: Coen Tempelaars
 */

#include "gtest/gtest.h"
#include "int/utilities/trace.hpp"
#include "ros/ros.h"

#include <boost/thread/thread.hpp>

#include "cShootPlanningRosStub.hpp"

// system under test
#include "int/adapters/cShootPlanningInterface.hpp"


// test utilities
cShootPlanningRosStub *shootPlanningRosStub;

static void spinThread()
{
    ros::spin();
}


// testCase: get/set active
TEST(ShootPlanningInterface, TestGetSetActive)
{
    // setup
    cShootPlanningInterface shootPlanningInterface;
    shootPlanningInterface.connect();
    shootPlanningRosStub->reset();

    EXPECT_FALSE(shootPlanningInterface.isEnabled());

    // execution
    shootPlanningInterface.enable();
    EXPECT_TRUE(shootPlanningInterface.isEnabled());

    shootPlanningInterface.enable();
    EXPECT_TRUE(shootPlanningInterface.isEnabled());

    shootPlanningInterface.disable();
    EXPECT_FALSE(shootPlanningInterface.isEnabled());

    shootPlanningInterface.disable();
    EXPECT_FALSE(shootPlanningInterface.isEnabled());

    shootPlanningInterface.enable();
    EXPECT_TRUE(shootPlanningInterface.isEnabled());

    // teardown
    shootPlanningRosStub->reset();
}


// testCase: shoot
TEST(ShootPlanningInterface, TestShoot)
{
    // setup
    cShootPlanningInterface shootPlanningInterface;
    shootPlanningInterface.connect();
    shootPlanningRosStub->reset();

    EXPECT_NEAR(0.000, shootPlanningRosStub->getLastShotStrength(), 0.0001);

    // execution
    float strength = 4.0;
    shootPlanningInterface.shoot(strength);

    // assertions/expectations
    EXPECT_NEAR(4.000, shootPlanningRosStub->getLastShotStrength(), 0.0001);

    // teardown
    shootPlanningRosStub->reset();
}

// MAIN
int main(int argc, char **argv)
{
    //Enable tracing
    teamplay::traceRedirect::getInstance().setAllTracesToStdout();

    // initialize
    ros::init(argc, argv, "shootPlanningInterfaceTest");

    // instantiate cWorldModelRosStub and start a ROS spinner thread to prevent deadlock within this process
    shootPlanningRosStub = new cShootPlanningRosStub();
    boost::thread st(&spinThread);

    // Run all the tests that were declared with TEST()
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

