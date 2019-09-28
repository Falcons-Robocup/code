 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ballTest.cpp
 *
 *  Created on: Nov 21, 2018
 *      Author: Coen Tempelaars
 */

#include <gtest/gtest.h>

#include "int/ball.hpp"


class ABall : public ::testing::Test
{
public:
    Ball ball;
};


TEST_F(ABall, HasPosition)
{
    auto position = Point3D(0.5, 1.0, 1.5);
    ball.setPosition(position);
    EXPECT_EQ(position, ball.getPosition());
}

TEST_F(ABall, HasVelocity)
{
    auto velocity = Vector3D(0.1, 0.2, 0.3);
    ball.setVelocity(velocity);
    EXPECT_EQ(velocity, ball.getVelocity());
}

TEST_F(ABall, CanBeTeleportedWithoutSpecifyingHeight)
{
    ball.teleport(0.2, 0.4);
    EXPECT_EQ(Point3D(0.2, 0.4, 0.0), ball.getPosition());
}

TEST_F(ABall, CanBeTeleported)
{
    ball.teleport(0.2, 0.4, 0.6);
    EXPECT_EQ(Point3D(0.2, 0.4, 0.6), ball.getPosition());
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
