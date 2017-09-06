 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRosAdapterTeamplayTest.cpp
 *
 *  Created on: Feb 16, 2016
 *      Author: Coen Tempelaars
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <boost/thread/thread.hpp>

#include "int/cRosAdapterTeamplay.hpp"
#include "int/cTeamplayControlInterface.hpp"
#include "int/cTeamplayServices.hpp"

#include "teamplay/s_park.h"
#include "teamplay/s_set_orientation.h"
#include "teamplay/s_stop.h"
#include "teamplay/s_teamplay_get_active.h"
#include "teamplay/s_teamplay_set_active.h"

using namespace testing;


// test utilities
static void spinThread()
{
    ros::spin();
}


class cTeamplayControlInterfaceMock : public cTeamplayControlInterface
{
public:
    cTeamplayControlInterfaceMock() {};
    ~cTeamplayControlInterfaceMock() {};

    MOCK_METHOD0(disable, void());
    MOCK_METHOD0(enable, void());
    MOCK_METHOD0(isActive, bool());
    MOCK_METHOD0(park, void());
    MOCK_METHOD1(setOrientation, void(orientationEnum));
    MOCK_METHOD0(stop, void());

};


class RosAdapterTeamplay : public testing::Test
{
public:
    //Construct mock
    cTeamplayControlInterfaceMock mock;

    //Construct ros adapter
    cRosAdapterTeamplay ros_tp = cRosAdapterTeamplay(&mock);

    void SetUp() {
        //Wait for all services to be alive
        ros::service::waitForService(TeamplayServices::s_park);
        ros::service::waitForService(TeamplayServices::s_set_orientation);
    }
};


TEST_F(RosAdapterTeamplay, Park)
{
    //Set expectation
    EXPECT_CALL(mock, park());

    //Do service call
    teamplay::s_park srv;
    ros::service::call(TeamplayServices::s_park, srv);
}


TEST_F(RosAdapterTeamplay, SetOrientationLeftToRight)
{
    //Set expectation
    EXPECT_CALL(mock, setOrientation(orientationEnum::LEFT_TO_RIGHT));

    //Do service call
    teamplay::s_set_orientation srv;
    srv.request.orientation = teamplay::s_set_orientation::Request::ORIENTATION_LEFT_TO_RIGHT;
    ros::service::call(TeamplayServices::s_set_orientation, srv);
}


TEST_F(RosAdapterTeamplay, SetOrientationRightToLeft)
{
    //Set expectation
    EXPECT_CALL(mock, setOrientation(orientationEnum::RIGHT_TO_LEFT));

    //Do service call
    teamplay::s_set_orientation srv;
    srv.request.orientation = teamplay::s_set_orientation::Request::ORIENTATION_RIGHT_TO_LEFT;
    ros::service::call(TeamplayServices::s_set_orientation, srv);
}


TEST_F(RosAdapterTeamplay, SetOrientationWithIllegalParameter)
{
    //Set expectation
    EXPECT_CALL(mock, setOrientation(_)).Times(0);

    //Do service call
    teamplay::s_set_orientation srv;
    srv.request.orientation = 12;
    ros::service::call(TeamplayServices::s_set_orientation, srv);
}


TEST_F(RosAdapterTeamplay, Stop)
{
    //Set expectation
    EXPECT_CALL(mock, stop());

    //Do service call
    teamplay::s_stop srv;
    ros::service::call(TeamplayServices::s_stop, srv);
}


TEST_F(RosAdapterTeamplay, IsActive)
{
    //Set expectation
    EXPECT_CALL(mock, isActive()).WillOnce(Return(true));

    //Do service call
    teamplay::s_teamplay_get_active srv;
    ros::service::call(TeamplayServices::s_teamplay_get_active, srv);

    ASSERT_TRUE(srv.response.active);
}


TEST_F(RosAdapterTeamplay, IsNotActive)
{
    //Set expectation
    EXPECT_CALL(mock, isActive()).WillOnce(Return(false));

    //Do service call
    teamplay::s_teamplay_get_active srv;
    ros::service::call(TeamplayServices::s_teamplay_get_active, srv);

    ASSERT_FALSE(srv.response.active);
}


TEST_F(RosAdapterTeamplay, SetActive)
{
    //Set expectation
    EXPECT_CALL(mock, enable());

    //Do service call
    teamplay::s_teamplay_set_active srv;
    srv.request.active = true;
    ros::service::call(TeamplayServices::s_teamplay_set_active, srv);
}


TEST_F(RosAdapterTeamplay, SetPassive)
{
    //Set expectation
    EXPECT_CALL(mock, disable());

    //Do service call
    teamplay::s_teamplay_set_active srv;
    srv.request.active = false;
    ros::service::call(TeamplayServices::s_teamplay_set_active, srv);
}


// MAIN
int main(int argc, char **argv)
{
    // initialize
    ros::init(argc, argv, "cRosAdapterTeamplayTest");

    // start a ROS spinner thread to prevent deadlock within this process
    boost::thread st(&spinThread);

    // Run all the tests that were declared with TEST()
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
