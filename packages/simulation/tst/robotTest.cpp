 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * robotTest.cpp
 *
 *  Created on: Nov 21, 2018
 *      Author: Coen Tempelaars
 */

#include <gtest/gtest.h>
#include "int/robot.hpp"

const static float SIMULATION_PERIOD_FOR_TEST = 0.2;

class ARobot : public ::testing::Test
{
public:
    ARobot()
    {
        robot.setBallHandlingModulePresent();
    }

    Robot robot;
};


TEST_F(ARobot, HasPosition)
{
    auto absolute_position = Position2D(-0.1, 0.3, 0.5);

    robot.setPlayingDirection(PlayingDirection::LEFT_TO_RIGHT);
    robot.setPosition(absolute_position);

    EXPECT_EQ(-0.1, robot.getPosition().x);
    EXPECT_EQ( 0.3, robot.getPosition().y);
    EXPECT_EQ( 0.5, robot.getPosition().phi);

    robot.setPlayingDirection(PlayingDirection::RIGHT_TO_LEFT);
    robot.setPosition(absolute_position);

    EXPECT_EQ(-0.1, robot.getPosition().x);
    EXPECT_EQ( 0.3, robot.getPosition().y);
    EXPECT_EQ( 0.5, robot.getPosition().phi);
}

TEST_F(ARobot, AcceptsPositionInFCS)
{
    auto fcs_position = Position2D(-0.1, 0.3, 0.5);

    robot.setPlayingDirection(PlayingDirection::LEFT_TO_RIGHT);
    robot.setPositionFCS(fcs_position);

    EXPECT_EQ(-0.1, robot.getPosition().x);
    EXPECT_EQ( 0.3, robot.getPosition().y);
    EXPECT_EQ( 0.5, robot.getPosition().phi);

    EXPECT_EQ(-0.1, robot.getPositionFCS().x);
    EXPECT_EQ( 0.3, robot.getPositionFCS().y);
    EXPECT_EQ( 0.5, robot.getPositionFCS().phi);

    robot.setPlayingDirection(PlayingDirection::RIGHT_TO_LEFT);
    robot.setPositionFCS(fcs_position);

    EXPECT_EQ( 0.1, robot.getPosition().x);
    EXPECT_EQ(-0.3, robot.getPosition().y);
    EXPECT_EQ( 0.5 + M_PI, robot.getPosition().phi);

    EXPECT_EQ(-0.1, robot.getPositionFCS().x);
    EXPECT_EQ( 0.3, robot.getPositionFCS().y);
    EXPECT_EQ( 0.5, robot.getPositionFCS().phi);
}

TEST_F(ARobot, HasVelocity)
{
    auto absolute_velocity = Velocity2D(-0.2, 0.4, 0.6);

    robot.setPlayingDirection(PlayingDirection::LEFT_TO_RIGHT);
    robot.setVelocity(absolute_velocity);

    EXPECT_EQ(-0.2, robot.getVelocity().x);
    EXPECT_EQ( 0.4, robot.getVelocity().y);
    EXPECT_EQ( 0.6, robot.getVelocity().phi);

    robot.setPlayingDirection(PlayingDirection::RIGHT_TO_LEFT);
    robot.setVelocity(absolute_velocity);

    EXPECT_EQ(-0.2, robot.getVelocity().x);
    EXPECT_EQ( 0.4, robot.getVelocity().y);
    EXPECT_EQ( 0.6, robot.getVelocity().phi);
}

TEST_F(ARobot, AcceptsVelocityInRCS)
{
    auto rcs_velocity = Velocity2D(-0.2, 0.4, 0.6);

    robot.setPlayingDirection(PlayingDirection::LEFT_TO_RIGHT);
    robot.setVelocityRCS(rcs_velocity);

    EXPECT_NEAR( 0.4, robot.getVelocity().x, 0.01);
    EXPECT_NEAR( 0.2, robot.getVelocity().y, 0.01);
    EXPECT_NEAR( 0.6, robot.getVelocity().phi, 0.01);

    EXPECT_NEAR( 0.4, robot.getVelocityFCS().x, 0.01);
    EXPECT_NEAR( 0.2, robot.getVelocityFCS().y, 0.01);
    EXPECT_NEAR( 0.6, robot.getVelocityFCS().phi, 0.01);

    robot.setPlayingDirection(PlayingDirection::RIGHT_TO_LEFT);
    robot.setVelocityRCS(rcs_velocity);

    EXPECT_NEAR( 0.4, robot.getVelocity().x, 0.01);
    EXPECT_NEAR( 0.2, robot.getVelocity().y, 0.01);
    EXPECT_NEAR( 0.6, robot.getVelocity().phi, 0.01);

    EXPECT_NEAR(-0.4, robot.getVelocityFCS().x, 0.01);
    EXPECT_NEAR(-0.2, robot.getVelocityFCS().y, 0.01);
    EXPECT_NEAR( 0.6, robot.getVelocityFCS().phi, 0.01);
}

TEST_F(ARobot, CanRecalculateItsPosition_LeftToRight)
{
    auto fcs_position = Position2D(0.0, -2.0, M_PI);
    auto rcs_velocity = Velocity2D(0.10, -0.06, -0.12);

    robot.setPlayingDirection(PlayingDirection::LEFT_TO_RIGHT);
    robot.setPositionFCS(fcs_position);
    robot.setVelocityRCS(rcs_velocity);

    EXPECT_NEAR( 0.0, robot.getPositionFCS().x, 0.01);
    EXPECT_NEAR(-2.0, robot.getPositionFCS().y, 0.01);
    EXPECT_NEAR( M_PI, robot.getPositionFCS().phi, 0.01);

    robot.recalculatePosition(SIMULATION_PERIOD_FOR_TEST);

    EXPECT_NEAR( 0.01, robot.getPositionFCS().x, 0.01);
    EXPECT_NEAR(-1.98, robot.getPositionFCS().y, 0.01);
    EXPECT_NEAR( M_PI - 0.03, robot.getPositionFCS().phi, 0.01);

    robot.recalculatePosition(SIMULATION_PERIOD_FOR_TEST);

    EXPECT_NEAR( 0.02, robot.getPositionFCS().x, 0.01);
    EXPECT_NEAR(-1.96, robot.getPositionFCS().y, 0.01);
    EXPECT_NEAR( M_PI - 0.05, robot.getPositionFCS().phi, 0.01);
}

TEST_F(ARobot, CanRecalculateItsPosition_RightToLeft)
{
    auto fcs_position = Position2D(0.0, -2.0, M_PI);
    auto rcs_velocity = Velocity2D(0.10, -0.06, -0.12);

    robot.setPlayingDirection(PlayingDirection::RIGHT_TO_LEFT);
    robot.setPositionFCS(fcs_position);
    robot.setVelocityRCS(rcs_velocity);

    EXPECT_NEAR( 0.0, robot.getPositionFCS().x, 0.01);
    EXPECT_NEAR(-2.0, robot.getPositionFCS().y, 0.01);
    EXPECT_NEAR( M_PI, robot.getPositionFCS().phi, 0.01);

    robot.recalculatePosition(SIMULATION_PERIOD_FOR_TEST);

    EXPECT_NEAR( 0.01, robot.getPositionFCS().x, 0.01);
    EXPECT_NEAR(-1.98, robot.getPositionFCS().y, 0.01);
    EXPECT_NEAR( M_PI - 0.03, robot.getPositionFCS().phi, 0.01);

    robot.recalculatePosition(SIMULATION_PERIOD_FOR_TEST);

    EXPECT_NEAR( 0.02, robot.getPositionFCS().x, 0.01);
    EXPECT_NEAR(-1.96, robot.getPositionFCS().y, 0.01);
    EXPECT_NEAR( M_PI - 0.05, robot.getPositionFCS().phi, 0.01);
}

TEST_F(ARobot, CanRecalculateItsVelocity_LeftToRight)
{
    auto fcs_position = Position2D(0.0, -2.0, M_PI);
    auto rcs_velocity = Velocity2D(0.10, -0.06, -0.12);

    robot.setPlayingDirection(PlayingDirection::LEFT_TO_RIGHT);
    robot.setPositionFCS(fcs_position);
    robot.setVelocityRCS(rcs_velocity);

    EXPECT_NEAR( 0.06, robot.getVelocityFCS().x, 0.01);
    EXPECT_NEAR( 0.10, robot.getVelocityFCS().y, 0.01);
    EXPECT_NEAR(-0.12, robot.getVelocityFCS().phi, 0.01);
}

TEST_F(ARobot, CanRecalculateItsVelocity_RightToLeft)
{
    auto fcs_position = Position2D(0.0, -2.0, M_PI);
    auto rcs_velocity = Velocity2D(0.10, -0.06, -0.12);

    robot.setPlayingDirection(PlayingDirection::RIGHT_TO_LEFT);
    robot.setPositionFCS(fcs_position);
    robot.setVelocityRCS(rcs_velocity);

    EXPECT_NEAR( 0.06, robot.getVelocityFCS().x, 0.01);
    EXPECT_NEAR( 0.10, robot.getVelocityFCS().y, 0.01);
    EXPECT_NEAR(-0.12, robot.getVelocityFCS().phi, 0.01);
}

TEST_F(ARobot, HasKickerHeight)
{
    float height = 0.75;
    robot.setKickerHeight(height);
    EXPECT_EQ(height, robot.getKickerHeight());
}

TEST_F(ARobot, HasKickerSpeed)
{
    float speed = 1.25;
    float scale = 0.5;
    robot.setKickerSpeed(speed, scale);
    EXPECT_EQ(speed * scale, robot.getKickerSpeed());
}

TEST_F(ARobot, ClaimsBallNearBallMouth)
{
    Point3D ballPosition = Point3D(0.2, 0.0, 0.0);
    EXPECT_FALSE(robot.canGrabBall(ballPosition));
    EXPECT_TRUE(robot.canKickBall(ballPosition));

    robot.enableBallHandlers();
    EXPECT_TRUE(robot.hasBallHandlersEnabled());
    EXPECT_TRUE(robot.canGrabBall(ballPosition));
    EXPECT_TRUE(robot.canKickBall(ballPosition));

    robot.disableBallHandlers();
    EXPECT_FALSE(robot.hasBallHandlersEnabled());
    EXPECT_FALSE(robot.canGrabBall(ballPosition));
    EXPECT_TRUE(robot.canKickBall(ballPosition));
}

TEST_F(ARobot, DoesNotClaimBallNotNearBallMouth)
{
    Point3D ballPosition = Point3D(0.2, 0.5, 0.0);
    EXPECT_FALSE(robot.canGrabBall(ballPosition));
    EXPECT_FALSE(robot.canKickBall(ballPosition));

    robot.enableBallHandlers();
    EXPECT_TRUE(robot.hasBallHandlersEnabled());
    EXPECT_FALSE(robot.canGrabBall(ballPosition));
    EXPECT_FALSE(robot.canKickBall(ballPosition));

    robot.disableBallHandlers();
    EXPECT_FALSE(robot.hasBallHandlersEnabled());
    EXPECT_FALSE(robot.canGrabBall(ballPosition));
    EXPECT_FALSE(robot.canKickBall(ballPosition));
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
