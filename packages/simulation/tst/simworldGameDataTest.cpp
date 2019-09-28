 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * simworldGameDataTest.cpp
 *
 *  Created on: Nov 21, 2018
 *      Author: Coen Tempelaars
 */

#include <gtest/gtest.h>
#include "int/simworldGameData.hpp"

const static float SIMULATION_PERIOD_FOR_TEST = 0.05;

class OneStaticRobot : public ::testing::Test
{
public:
    OneStaticRobot()
    {
        gameData.team = { {TeamID::A, { {RobotID::r1, Robot()} } } };

        gameData.team[TeamID::A][RobotID::r1].setPosition(Position2D(1.0, 1.0, 0.0));
        gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(0.0, 0.0, 0.0));
        gameData.team[TeamID::A][RobotID::r1].setBallHandlingModulePresent();
        gameData.team[TeamID::A][RobotID::r1].enableBallHandlers();
    }

    SimworldGameData gameData;
};


TEST_F(OneStaticRobot, BallAndRobotDoNotCollide)
{
    gameData.ball.setPosition(Point3D(0.7, 0.7, 0.0));
    gameData.ball.setVelocity(Vector3D(0.1, 0.1, 0.0));

    gameData.recalculateWorld(SIMULATION_PERIOD_FOR_TEST);

    EXPECT_NEAR(0.7, gameData.ball.getPosition().x, 0.1);
    EXPECT_NEAR(0.7, gameData.ball.getPosition().y, 0.1);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getPosition().z);

    EXPECT_NEAR(0.1, gameData.ball.getVelocity().x, 0.1);
    EXPECT_NEAR(0.1, gameData.ball.getVelocity().y, 0.1);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().z);

    EXPECT_NEAR(1.0, gameData.team[TeamID::A][RobotID::r1].getPosition().x, 0.001);
    EXPECT_NEAR(1.0, gameData.team[TeamID::A][RobotID::r1].getPosition().y, 0.001);

    EXPECT_FALSE(gameData.team[TeamID::A][RobotID::r1].canKickBall(gameData.ball.getPosition()));
}


TEST_F(OneStaticRobot, BallAndRobotCollide)
{
    gameData.ball.setPosition(Point3D(0.8, 0.8, 0.0));
    gameData.ball.setVelocity(Vector3D(0.1, 0.1, 0.0));

    gameData.recalculateWorld(SIMULATION_PERIOD_FOR_TEST);

    EXPECT_NEAR(0.6, gameData.ball.getPosition().x, 0.1);
    EXPECT_NEAR(0.6, gameData.ball.getPosition().y, 0.1);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getPosition().z);

    EXPECT_NEAR(-0.1, gameData.ball.getVelocity().x, 0.1);
    EXPECT_NEAR(-0.1, gameData.ball.getVelocity().y, 0.1);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().z);

    EXPECT_NEAR(1.0, gameData.team[TeamID::A][RobotID::r1].getPosition().x, 0.001);
    EXPECT_NEAR(1.0, gameData.team[TeamID::A][RobotID::r1].getPosition().y, 0.001);

    EXPECT_FALSE(gameData.team[TeamID::A][RobotID::r1].canKickBall(gameData.ball.getPosition()));
}


TEST_F(OneStaticRobot, BallRollsIntoBallHandlers)
{
    gameData.ball.setPosition(Point3D(1.25, 1.15, 0.0));
    gameData.ball.setVelocity(Vector3D(-0.12, -0.08, 0.0));

    gameData.recalculateWorld(SIMULATION_PERIOD_FOR_TEST);

    EXPECT_NEAR(1.25, gameData.ball.getPosition().x, 0.1);
    EXPECT_NEAR(1.0, gameData.ball.getPosition().y, 0.1);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getPosition().z);

    EXPECT_NEAR(0.0, gameData.ball.getVelocity().x, 0.1);
    EXPECT_NEAR(0.0, gameData.ball.getVelocity().y, 0.1);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().z);

    EXPECT_NEAR(1.0, gameData.team[TeamID::A][RobotID::r1].getPosition().x, 0.001);
    EXPECT_NEAR(1.0, gameData.team[TeamID::A][RobotID::r1].getPosition().y, 0.001);

    EXPECT_TRUE(gameData.team[TeamID::A][RobotID::r1].canKickBall(gameData.ball.getPosition()));
}


class OneMovingRobot : public ::testing::Test
{
public:
    OneMovingRobot()
    {
        gameData.team = { {TeamID::A, { {RobotID::r1, Robot()} } } };

        gameData.team[TeamID::A][RobotID::r1].setPosition(Position2D(1.0, 1.0, 0.0));
        gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(-0.1, -0.1, 0.0));
        gameData.team[TeamID::A][RobotID::r1].setBallHandlingModulePresent();
        gameData.team[TeamID::A][RobotID::r1].enableBallHandlers();
    }

    SimworldGameData gameData;
};


TEST_F(OneMovingRobot, BallAndRobotDoNotCollide)
{
    gameData.ball.setPosition(Point3D(0.7, 0.7, 0.0));
    gameData.ball.setVelocity(Vector3D(0.1, 0.1, 0.0));

    gameData.recalculateWorld(SIMULATION_PERIOD_FOR_TEST);

    EXPECT_NEAR(0.7, gameData.ball.getPosition().x, 0.1);
    EXPECT_NEAR(0.7, gameData.ball.getPosition().y, 0.1);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getPosition().z);

    EXPECT_NEAR(0.1, gameData.ball.getVelocity().x, 0.1);
    EXPECT_NEAR(0.1, gameData.ball.getVelocity().y, 0.1);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().z);

    EXPECT_NEAR(0.995, gameData.team[TeamID::A][RobotID::r1].getPosition().x, 0.001);
    EXPECT_NEAR(0.995, gameData.team[TeamID::A][RobotID::r1].getPosition().y, 0.001);

    EXPECT_FALSE(gameData.team[TeamID::A][RobotID::r1].canKickBall(gameData.ball.getPosition()));
}


TEST_F(OneMovingRobot, BallAndRobotCollide)
{
    gameData.ball.setPosition(Point3D(0.8, 0.8, 0.0));
    gameData.ball.setVelocity(Vector3D(0.1, 0.1, 0.0));

    gameData.recalculateWorld(SIMULATION_PERIOD_FOR_TEST);

    EXPECT_NEAR(0.7, gameData.ball.getPosition().x, 0.1);
    EXPECT_NEAR(0.7, gameData.ball.getPosition().y, 0.1);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getPosition().z);

    EXPECT_NEAR(-0.3, gameData.ball.getVelocity().x, 0.1);
    EXPECT_NEAR(-0.3, gameData.ball.getVelocity().y, 0.1);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().z);

    EXPECT_NEAR(0.995, gameData.team[TeamID::A][RobotID::r1].getPosition().x, 0.001);
    EXPECT_NEAR(0.995, gameData.team[TeamID::A][RobotID::r1].getPosition().y, 0.001);

    EXPECT_FALSE(gameData.team[TeamID::A][RobotID::r1].canKickBall(gameData.ball.getPosition()));
}


TEST_F(OneMovingRobot, RobotGrabsIncomingBall)
{
    gameData.ball.setPosition(Point3D(0.8, 0.8, 0.0));
    gameData.ball.setVelocity(Vector3D(0.1, 0.1, 0.0));

    /* Turn the robot facing the ball */
    gameData.team[TeamID::A][RobotID::r1].setPosition(Position2D(1.0, 1.0, 1.25*M_PI));
    gameData.recalculateWorld(SIMULATION_PERIOD_FOR_TEST);

    EXPECT_NEAR(0.8, gameData.ball.getPosition().x, 0.1);
    EXPECT_NEAR(0.8, gameData.ball.getPosition().y, 0.1);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getPosition().z);

    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().x);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().y);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().z);

    EXPECT_NEAR(0.995, gameData.team[TeamID::A][RobotID::r1].getPosition().x, 0.001);
    EXPECT_NEAR(0.995, gameData.team[TeamID::A][RobotID::r1].getPosition().y, 0.001);

    EXPECT_TRUE(gameData.team[TeamID::A][RobotID::r1].canKickBall(gameData.ball.getPosition()));
}


TEST_F(OneMovingRobot, RobotOvertakesBallAndHitsIt)
{
    gameData.ball.setPosition(Point3D(0.95, 0.95, 0.0));
    gameData.ball.setVelocity(Vector3D(-0.05, -0.05, 0.0));

    gameData.recalculateWorld(SIMULATION_PERIOD_FOR_TEST);

    EXPECT_NEAR(0.5, gameData.ball.getPosition().x, 0.1);
    EXPECT_NEAR(0.5, gameData.ball.getPosition().y, 0.1);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getPosition().z);

    EXPECT_NEAR(-0.1, gameData.ball.getVelocity().x, 0.1);
    EXPECT_NEAR(-0.1, gameData.ball.getVelocity().y, 0.1);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().z);

    EXPECT_NEAR(0.995, gameData.team[TeamID::A][RobotID::r1].getPosition().x, 0.001);
    EXPECT_NEAR(0.995, gameData.team[TeamID::A][RobotID::r1].getPosition().y, 0.001);

    EXPECT_FALSE(gameData.team[TeamID::A][RobotID::r1].canKickBall(gameData.ball.getPosition()));
}


TEST_F(OneMovingRobot, RobotOvertakesBallAndGrabsIt)
{
    gameData.ball.setPosition(Point3D(0.95, 0.95, 0.0));
    gameData.ball.setVelocity(Vector3D(-0.05, -0.05, 0.0));

    /* Turn the robot facing the ball */
    gameData.team[TeamID::A][RobotID::r1].setPosition(Position2D(1.0, 1.0, 1.25*M_PI));
    gameData.recalculateWorld(SIMULATION_PERIOD_FOR_TEST);

    EXPECT_NEAR(0.8, gameData.ball.getPosition().x, 0.1);
    EXPECT_NEAR(0.8, gameData.ball.getPosition().y, 0.1);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getPosition().z);

    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().x);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().y);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().z);

    EXPECT_NEAR(0.995, gameData.team[TeamID::A][RobotID::r1].getPosition().x, 0.001);
    EXPECT_NEAR(0.995, gameData.team[TeamID::A][RobotID::r1].getPosition().y, 0.001);

    EXPECT_TRUE(gameData.team[TeamID::A][RobotID::r1].canKickBall(gameData.ball.getPosition()));
}


class OneRobotWithBall : public ::testing::Test
{
public:
    OneRobotWithBall()
    {
        gameData.team = { {TeamID::A, { {RobotID::r1, Robot()} } } };

        gameData.team[TeamID::A][RobotID::r1].setPosition(Position2D(1.0, 1.0, 0.0));
        gameData.team[TeamID::A][RobotID::r1].setBallHandlingModulePresent();
        gameData.team[TeamID::A][RobotID::r1].enableBallHandlers();

        gameData.ball.setPosition(Point3D(1.25, 1.0, 0.0));
    }

    SimworldGameData gameData;
};


TEST_F(OneRobotWithBall, TurnsWithoutLosingTheBall)
{
    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(0.0, 0.0, 3.0));
    gameData.recalculateWorld(SIMULATION_PERIOD_FOR_TEST);

    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().x);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().y);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().z);

    EXPECT_NEAR(1.0, gameData.team[TeamID::A][RobotID::r1].getPosition().x, 0.001);
    EXPECT_NEAR(1.0, gameData.team[TeamID::A][RobotID::r1].getPosition().y, 0.001);
    EXPECT_NEAR(0.15, gameData.team[TeamID::A][RobotID::r1].getPosition().phi, 0.1);

    EXPECT_TRUE(gameData.team[TeamID::A][RobotID::r1].canGrabBall(gameData.ball.getPosition()));
    EXPECT_TRUE(gameData.team[TeamID::A][RobotID::r1].canKickBall(gameData.ball.getPosition()));
}


TEST_F(OneRobotWithBall, KicksTheBall)
{
    gameData.team[TeamID::A][RobotID::r1].setKickerSpeed(100.0, 1.0);
    gameData.recalculateWorld(SIMULATION_PERIOD_FOR_TEST);

    EXPECT_FLOAT_EQ(100.0, gameData.ball.getVelocity().x);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().y);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().z);

    EXPECT_NEAR(1.0, gameData.team[TeamID::A][RobotID::r1].getPosition().x, 0.001);
    EXPECT_NEAR(1.0, gameData.team[TeamID::A][RobotID::r1].getPosition().y, 0.001);

    EXPECT_FALSE(gameData.team[TeamID::A][RobotID::r1].canGrabBall(gameData.ball.getPosition()));
    EXPECT_FALSE(gameData.team[TeamID::A][RobotID::r1].canKickBall(gameData.ball.getPosition()));
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
