 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Author: ctempela
 * Creation: 2015-11-19
 *
 * Unit test for Pose2D class
 */

#include "ext/pose2d.hpp"

#include <gtest/gtest.h>
#include <math.h>


TEST(TestPose2D, testConstructAndAssignment)
{
  geometry::Pose2D pose1;
  EXPECT_EQ(0.0, pose1.getX());
  EXPECT_EQ(0.0, pose1.getY());
  EXPECT_EQ(0.0, pose1.getPhi());

  geometry::Pose2D pose2(1.0, 2.0, 3.0);
  EXPECT_EQ(1.0, pose2.getX());
  EXPECT_EQ(2.0, pose2.getY());
  EXPECT_EQ(3.0, pose2.getPhi());

  geometry::Pose2D pose3(pose2);
  EXPECT_EQ(1.0, pose3.getX());
  EXPECT_EQ(2.0, pose3.getY());
  EXPECT_EQ(3.0, pose3.getPhi());

  pose1 = pose3;
  EXPECT_EQ(1.0, pose1.getX());
  EXPECT_EQ(2.0, pose1.getY());
  EXPECT_EQ(3.0, pose1.getPhi());
}


TEST(TestPose2D, testTeleport)
{
  geometry::Pose2D pose1;

  pose1.teleport(4.0, 3.0, 2.0);
  EXPECT_EQ(4.0, pose1.getX());
  EXPECT_EQ(3.0, pose1.getY());
  EXPECT_EQ(2.0, pose1.getPhi());
}


TEST(TestPose2D, testTurn)
{
  geometry::Pose2D pose1(0.0, 0.0, M_PI);
  EXPECT_NEAR(M_PI, pose1.getPhi(), 0.00001);

  pose1.turn(0.25 * M_PI);
  EXPECT_NEAR((1.25 * M_PI), pose1.getPhi(), 0.00001);

  pose1.turn(5 * M_PI);
  EXPECT_NEAR((0.25 * M_PI), pose1.getPhi(), 0.00001);

  pose1.turn(-0.6 * M_PI);
  EXPECT_NEAR((1.65 * M_PI), pose1.getPhi(), 0.00001);
}


TEST(TestPose2D, testShift)
{
	geometry::Pose2D pose1(1.0, 2.0, 3.0);
	EXPECT_EQ(1.0, pose1.getX());
	EXPECT_EQ(2.0, pose1.getY());
	EXPECT_EQ(3.0, pose1.getPhi());

	pose1.shift(5.0, -5.0);
	EXPECT_EQ(6.0, pose1.getX());
	EXPECT_EQ(-3.0, pose1.getY());
	EXPECT_EQ(3.0, pose1.getPhi());

	pose1.shift(-6.0, 0.0);
	EXPECT_EQ(0.0, pose1.getX());
	EXPECT_EQ(-3.0, pose1.getY());
	EXPECT_EQ(3.0, pose1.getPhi());
}


TEST(TestPose2D, testConstructPoseWithAngleOutOfLimits)
{
	  geometry::Pose2D pose1(-1.0, -2.0, (-5.0 * M_PI));
	  EXPECT_EQ(-1.0, pose1.getX());
	  EXPECT_EQ(-2.0, pose1.getY());
	  EXPECT_NEAR((1.0 * M_PI), pose1.getPhi(), 0.00001);

	  geometry::Pose2D pose2(1.0, 2.0, (3.5 * M_PI));
	  EXPECT_EQ(1.0, pose2.getX());
	  EXPECT_EQ(2.0, pose2.getY());
	  EXPECT_NEAR((1.5 * M_PI), pose2.getPhi(), 0.00001);

	  geometry::Pose2D pose3(pose2);
	  EXPECT_EQ(1.0, pose3.getX());
	  EXPECT_EQ(2.0, pose3.getY());
	  EXPECT_NEAR((1.5 * M_PI), pose3.getPhi(), 0.00001);

	  pose1 = pose3;
	  EXPECT_EQ(1.0, pose1.getX());
	  EXPECT_EQ(2.0, pose1.getY());
	  EXPECT_NEAR((1.5 * M_PI), pose1.getPhi(), 0.00001);
}


TEST(TestPose2D, testTransformFCS2RCS)
{
	geometry::Pose2D robot1(0.5, 0.5, (0.5 * M_PI));
	geometry::Pose2D robot2(-0.5, -5.0, (1.5 * M_PI));
	geometry::Pose2D robot3(3.0, 0.0, (0.75 * M_PI));

	geometry::Pose2D obstacle(2.0, 1.0, 0.0); // in FCS
	obstacle.transformFCS2RCS(robot1);
	EXPECT_NEAR(1.5, obstacle.getX(), 0.00001);
	EXPECT_NEAR(0.5, obstacle.getY(), 0.00001);
	EXPECT_NEAR(((obstacle.getPhi() < M_PI) ? (0.0) : (2 * M_PI)), obstacle.getPhi(), 0.00001); // phi is either near 0.0 or near 2pi

	obstacle.teleport(2.0, 1.0, 0.0); // in FCS
	obstacle.transformFCS2RCS(robot2);
	EXPECT_NEAR(-2.5, obstacle.getX(), 0.00001);
	EXPECT_NEAR(-6.0, obstacle.getY(), 0.00001);
	EXPECT_NEAR(M_PI, obstacle.getPhi(), 0.00001);

	obstacle.teleport(2.0, 1.0, 0.0); // in FCS
	obstacle.transformFCS2RCS(robot3);
	EXPECT_NEAR(0.0, obstacle.getX(), 0.00001);
	EXPECT_NEAR(sqrt(2.0), obstacle.getY(), 0.00001);
	EXPECT_NEAR((1.75 * M_PI), obstacle.getPhi(), 0.00001);
}


TEST(TestPose2D, testTransformRCS2FCS)
{
	geometry::Pose2D robot1(0.5, 0.5, (0.5 * M_PI));
	geometry::Pose2D robot2(-0.5, -5.0, (1.5 * M_PI));
	geometry::Pose2D robot3(3.0, 0.0, (0.75 * M_PI));

	geometry::Pose2D obstacle(1.5, 0.5, 0.0); // in RCS for robot 1
	obstacle.transformRCS2FCS(robot1);
	EXPECT_NEAR(2.0, obstacle.getX(), 0.00001);
	EXPECT_NEAR(1.0, obstacle.getY(), 0.00001);
	EXPECT_NEAR(((obstacle.getPhi() < M_PI) ? (0.0) : (2 * M_PI)), obstacle.getPhi(), 0.00001);

	obstacle.teleport(-2.5, -6.0, M_PI);
	obstacle.transformRCS2FCS(robot2);
	EXPECT_NEAR(2.0, obstacle.getX(), 0.00001);
	EXPECT_NEAR(1.0, obstacle.getY(), 0.00001);
	EXPECT_NEAR(((obstacle.getPhi() < M_PI) ? (0.0) : (2 * M_PI)), obstacle.getPhi(), 0.00001);

	obstacle.teleport(0.0, sqrt(2.0), (1.75 * M_PI));
	obstacle.transformRCS2FCS(robot3);
	EXPECT_NEAR(2.0, obstacle.getX(), 0.00001);
	EXPECT_NEAR(1.0, obstacle.getY(), 0.00001);
	EXPECT_NEAR(((obstacle.getPhi() < M_PI) ? (0.0) : (2 * M_PI)), obstacle.getPhi(), 0.00001);
}


TEST(TestPose2D, testTransformACS2FCS)
{
	geometry::Pose2D obstacle(1.5, 0.5, 0.0); // in FCS
	obstacle.transformACS2FCS(true);
	EXPECT_NEAR(1.5, obstacle.getX(), 0.00001);
	EXPECT_NEAR(0.5, obstacle.getY(), 0.00001);
	EXPECT_NEAR(((obstacle.getPhi() < M_PI) ? (0.0) : (2 * M_PI)), obstacle.getPhi(), 0.00001);

	obstacle.teleport(1.5, 0.5, 0.0); // in FCS
	obstacle.transformACS2FCS(false);
	EXPECT_NEAR(-1.5, obstacle.getX(), 0.00001);
	EXPECT_NEAR(-0.5, obstacle.getY(), 0.00001);
	EXPECT_NEAR(M_PI, obstacle.getPhi(), 0.00001);
}


TEST(TestPose2D, testTransformFCS2ACS)
{
	geometry::Pose2D obstacle(1.5, 0.5, 0.0); // in FCS
	obstacle.transformFCS2ACS(true);
	EXPECT_NEAR(1.5, obstacle.getX(), 0.00001);
	EXPECT_NEAR(0.5, obstacle.getY(), 0.00001);
	EXPECT_NEAR(((obstacle.getPhi() < M_PI) ? (0.0) : (2 * M_PI)), obstacle.getPhi(), 0.00001);

	obstacle.teleport(1.5, 0.5, 0.0); // in FCS
	obstacle.transformFCS2ACS(false);
	EXPECT_NEAR(-1.5, obstacle.getX(), 0.00001);
	EXPECT_NEAR(-0.5, obstacle.getY(), 0.00001);
	EXPECT_NEAR(M_PI, obstacle.getPhi(), 0.00001);
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
