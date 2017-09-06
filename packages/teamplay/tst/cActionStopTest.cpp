 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionStopTest.cpp
 * 
 *
 *  Created on: Apr 24, 2016
 *      Author: Coen Tempelaars
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

/* Include trace utility */
#include "int/utilities/trace.hpp"

// SUT dependencies
#include "int/adapters/cHALInterface.hpp"
#include "int/adapters/cPathPlanningInterface.hpp"
#include "int/cWorldModelInterface.hpp"
#include "mocks/worldModelUpdated.hpp"

// SUT
#include "int/actions/cActionStop.hpp"


using namespace testing;


class cHALInterfaceMock : public cHALInterface
{
public:
    cHALInterfaceMock() {};
    ~cHALInterfaceMock() {};

    MOCK_METHOD0(connect, void());
    MOCK_METHOD0(disableBallhandlers, void());
    MOCK_METHOD0(enableBallhandlers, void());
    MOCK_METHOD0(areBallhandlersEnabled, bool());
};

class cPathPlanningInterfaceMock : public cPathPlanningInterface
{
public:
    cPathPlanningInterfaceMock() {};
    ~cPathPlanningInterfaceMock() {};

    MOCK_METHOD0(connect, void());
    MOCK_METHOD0(disable, void());
    MOCK_METHOD0(enable, void());
    MOCK_METHOD0(isEnabled, bool());
    MOCK_METHOD3(moveTo, void(geometry::Pose2D&, const std::vector<polygon2D>&, const std::string&));
};


class ActionStop : public Test
{
public:
    //Construct mocks
    cHALInterfaceMock halInterfaceMock;
    cPathPlanningInterfaceMock ppInterfaceMock;

    //Construct SUT
    cActionStop actionStop;

    void SetUp() {
        ON_CALL(halInterfaceMock, areBallhandlersEnabled()).WillByDefault(Return(true));
        ON_CALL(ppInterfaceMock, isEnabled()).WillByDefault(Return(true));

        actionStop.setHALInterface( &halInterfaceMock );
        actionStop.setPathPlanningInterface( &ppInterfaceMock );
    }
};

TEST_F(ActionStop, Stop)
{
    Position2D own_position;
    cWorldModelInterface::getInstance().getOwnLocation( own_position );

    geometry::Pose2D expected_pose (own_position.x, own_position.y, own_position.phi);
    std::string expected_motionProfile = "normal";

    EXPECT_CALL(halInterfaceMock, disableBallhandlers());
    EXPECT_CALL(ppInterfaceMock, moveTo(expected_pose, _, expected_motionProfile));
    EXPECT_CALL(ppInterfaceMock, disable());
    actionStop.stopRobot();
}


int main(int argc, char **argv)
{
    //Enable tracing
    teamplay::traceRedirect::getInstance().setAllTracesToStdout();

    ros::init(argc, argv, "actionStopTest");
    InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

