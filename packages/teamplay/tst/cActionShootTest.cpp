 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionShootTest.cpp
 * 
 *
 *  Created on: Apr 23, 2016
 *      Author: Coen Tempelaars
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

// SUT dependencies
#include "int/adapters/cShootPlanningInterface.hpp"

// SUT
#include "int/actions/cActionShoot.hpp"


using namespace testing;


class cShootPlanningInterfaceMock : public cShootPlanningInterface
{
public:
    cShootPlanningInterfaceMock() {};
    ~cShootPlanningInterfaceMock() {};

    MOCK_METHOD0(connect, void());
    MOCK_METHOD0(disable, void());
    MOCK_METHOD0(enable, void());
    MOCK_METHOD0(isEnabled, bool());
    MOCK_METHOD1(shoot, void(float));
    MOCK_METHOD0(lobShot, void());
};


class ActionShoot : public Test
{
public:
    //Construct mock
    cShootPlanningInterfaceMock mock;

    //Construct SUT
    cActionShoot actionShoot;
};


// No tests defined yet...


int main(int argc, char **argv)
{
    ros::init(argc, argv, "actionShootTest");
    InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
