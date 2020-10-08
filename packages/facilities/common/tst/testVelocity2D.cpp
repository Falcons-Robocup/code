 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Author: ctempela
 * Creation: 2015-11-25
 *
 * Unit test for Velocity2D class
 */

#include "ext/velocity2d.hpp"

#include <gtest/gtest.h>
#include <math.h>


TEST(TestVelocity2D, testConstructAndAssignment)
{
  geometry::Velocity2D vel1;
  EXPECT_EQ(0.0, vel1.getX());
  EXPECT_EQ(0.0, vel1.getY());
  EXPECT_EQ(0.0, vel1.getPhi());

  geometry::Velocity2D vel2(1.0, 2.0, 3.0);
  EXPECT_EQ(1.0, vel2.getX());
  EXPECT_EQ(2.0, vel2.getY());
  EXPECT_EQ(3.0, vel2.getPhi());

  geometry::Velocity2D vel3(vel2);
  EXPECT_EQ(1.0, vel3.getX());
  EXPECT_EQ(2.0, vel3.getY());
  EXPECT_EQ(3.0, vel3.getPhi());

  vel1 = vel3;
  EXPECT_EQ(1.0, vel1.getX());
  EXPECT_EQ(2.0, vel1.getY());
  EXPECT_EQ(3.0, vel1.getPhi());
}


TEST(TestVelocity2D, testAccelerate)
{
  geometry::Velocity2D vel1(0.0, 0.0, 0.0);
  EXPECT_EQ(0.0, vel1.getX());
  EXPECT_EQ(0.0, vel1.getY());
  EXPECT_NEAR(0.0, vel1.getPhi(), 0.00001);

  vel1.accelerate(1.0, 2.0, (0.5 * M_PI));
  EXPECT_EQ(1.0, vel1.getX());
  EXPECT_EQ(2.0, vel1.getY());
  EXPECT_NEAR((0.5 * M_PI), vel1.getPhi(), 0.00001);

  vel1.accelerate(-1.5, -2.5, (-1.0 * M_PI));
  EXPECT_EQ(-0.5, vel1.getX());
  EXPECT_EQ(-0.5, vel1.getY());
  EXPECT_NEAR((-0.5 * M_PI), vel1.getPhi(), 0.00001);

  vel1.accelerate(0.0, 0.0, (5 * M_PI));
  EXPECT_EQ(-0.5, vel1.getX());
  EXPECT_EQ(-0.5, vel1.getY());
  EXPECT_NEAR((4.5 * M_PI), vel1.getPhi(), 0.00001);
}


TEST(TestVelocity2D, testTransformFCS2RCS)
{
    geometry::Pose2D robot1(0.5, 0.5, (0.5 * M_PI));
    geometry::Pose2D robot2(-0.5, -5.0, (1.5 * M_PI));
    geometry::Pose2D robot3(3.0, 0.0, (0.75 * M_PI));

    {
        geometry::Velocity2D obstacle(2.0, 1.0, (0.5 * M_PI)); // in FCS
        obstacle.transformFCS2RCS(robot1);
        EXPECT_NEAR(2.0, obstacle.getX(), 0.00001);
        EXPECT_NEAR(1.0, obstacle.getY(), 0.00001);
        EXPECT_NEAR((0.5 * M_PI), obstacle.getPhi(), 0.00001);
    }

    {
        geometry::Velocity2D obstacle(2.0, 1.0, (1.5 * M_PI)); // in FCS
        obstacle.transformFCS2RCS(robot2);
        EXPECT_NEAR(-2.0, obstacle.getX(), 0.00001);
        EXPECT_NEAR(-1.0, obstacle.getY(), 0.00001);
        EXPECT_NEAR((1.5 * M_PI), obstacle.getPhi(), 0.00001);
    }

    {
        geometry::Velocity2D obstacle(2.0, 2.0, (5.0 * M_PI)); // in FCS
        obstacle.transformFCS2RCS(robot3);
        EXPECT_NEAR(sqrt(8.0), obstacle.getX(), 0.00001);
        EXPECT_NEAR(0.0, obstacle.getY(), 0.00001);
        EXPECT_NEAR((5.0 * M_PI), obstacle.getPhi(), 0.00001);
    }
}


TEST(TestVelocity2D, testTransformRCS2FCS)
{
    geometry::Pose2D robot1(0.5, 0.5, (0.5 * M_PI));
    geometry::Pose2D robot2(-0.5, -5.0, (1.5 * M_PI));
    geometry::Pose2D robot3(3.0, 0.0, (0.75 * M_PI));

    {
        geometry::Velocity2D obstacle(2.0, 1.0, (0.5 * M_PI));
        obstacle.transformRCS2FCS(robot1);
        EXPECT_NEAR(2.0, obstacle.getX(), 0.00001);
        EXPECT_NEAR(1.0, obstacle.getY(), 0.00001);
        EXPECT_NEAR((0.5 * M_PI), obstacle.getPhi(), 0.00001);
    }

    {
        geometry::Velocity2D obstacle(-2.0, -1.0, (1.5 * M_PI));
        obstacle.transformRCS2FCS(robot2);
        EXPECT_NEAR(2.0, obstacle.getX(), 0.00001);
        EXPECT_NEAR(1.0, obstacle.getY(), 0.00001);
        EXPECT_NEAR((1.5 * M_PI), obstacle.getPhi(), 0.00001);
    }

    {
        geometry::Velocity2D obstacle(sqrt(8.0), 0.0, (5.0 * M_PI));
        obstacle.transformRCS2FCS(robot3);
        EXPECT_NEAR(2.0, obstacle.getX(), 0.00001);
        EXPECT_NEAR(2.0, obstacle.getY(), 0.00001);
        EXPECT_NEAR((5.0 * M_PI), obstacle.getPhi(), 0.00001);
    }
}


TEST(TestVelocity2D, testTransformACS2FCS)
{
    {
        geometry::Velocity2D obstacle(1.5, 0.5, M_PI); // in FCS
        obstacle.transformACS2FCS(true);
        EXPECT_NEAR(1.5, obstacle.getX(), 0.00001);
        EXPECT_NEAR(0.5, obstacle.getY(), 0.00001);
        EXPECT_NEAR(M_PI, obstacle.getPhi(), 0.00001);
    }

    {
        geometry::Velocity2D obstacle(1.5, 0.5, M_PI); // in FCS
        obstacle.transformACS2FCS(false);
        EXPECT_NEAR(-1.5, obstacle.getX(), 0.00001);
        EXPECT_NEAR(-0.5, obstacle.getY(), 0.00001);
        EXPECT_NEAR(M_PI, obstacle.getPhi(), 0.00001);
    }
}


TEST(TestVelocity2D, testTransformFCS2ACS)
{
    {
        geometry::Velocity2D obstacle(1.5, 0.5, M_PI); // in FCS
        obstacle.transformFCS2ACS(true);
        EXPECT_NEAR(1.5, obstacle.getX(), 0.00001);
        EXPECT_NEAR(0.5, obstacle.getY(), 0.00001);
        EXPECT_NEAR(M_PI, obstacle.getPhi(), 0.00001);
    }

    {
        geometry::Velocity2D obstacle(1.5, 0.5, M_PI); // in FCS
        obstacle.transformFCS2ACS(false);
        EXPECT_NEAR(-1.5, obstacle.getX(), 0.00001);
        EXPECT_NEAR(-0.5, obstacle.getY(), 0.00001);
        EXPECT_NEAR(M_PI, obstacle.getPhi(), 0.00001);
    }
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
