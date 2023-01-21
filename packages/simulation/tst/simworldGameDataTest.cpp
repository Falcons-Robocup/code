// Copyright 2018-2021 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * simworldGameDataTest.cpp
 *
 *  Created on: Nov 21, 2018
 *      Author: Coen Tempelaars
 */

#include <gtest/gtest.h>
#include "int/simworldGameData.hpp"
#include "int/gameDataFactory.hpp"
#include "tracing.hpp"


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
    TRACE_FUNCTION("");

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

    EXPECT_FALSE(gameData.team[TeamID::A][RobotID::r1].hasBall());
}


TEST_F(OneStaticRobot, BallAndRobotCollide)
{
    TRACE_FUNCTION("");

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

    EXPECT_FALSE(gameData.team[TeamID::A][RobotID::r1].hasBall());
}

TEST_F(OneStaticRobot, BallRollsIntoBallHandlers)
{
    TRACE_FUNCTION("");

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

    EXPECT_TRUE(gameData.team[TeamID::A][RobotID::r1].hasBall());
}


class AGame : public ::testing::Test
{
public:
    AGame()
    :
        gameData(GameDataFactory::createGameData(5, 5))
    {}

    SimworldGameData gameData;
};

TEST_F(AGame, NoRobotHasBall)
{
    EXPECT_FALSE(gameData.anyRobotHasBall());
    EXPECT_EQ(boost::none, gameData.getTeamWithBall());
    EXPECT_EQ(boost::none, gameData.getRobotWithBall(TeamID::A));
    EXPECT_EQ(boost::none, gameData.getRobotWithBall(TeamID::B));
}

TEST_F(AGame, OneRobotHasBall)
{
    gameData.team[TeamID::A][RobotID::r1].setPosition(Position2D(-3.0, -5.0,  0.5 * M_PI));
    gameData.team[TeamID::A][RobotID::r1].setBallHandlingModulePresent();
    gameData.team[TeamID::A][RobotID::r1].enableBallHandlers();
    gameData.ball.setLocation(Point2D(-3.0, -4.8));

    gameData.recalculateWorld(SIMULATION_PERIOD_FOR_TEST);

    EXPECT_TRUE(gameData.anyRobotHasBall());
    EXPECT_EQ(TeamID::A, gameData.getTeamWithBall());
    EXPECT_EQ(RobotID::r1, gameData.getRobotWithBall(TeamID::A));
    EXPECT_EQ(boost::none, gameData.getRobotWithBall(TeamID::B));
}

TEST_F(AGame, NoRobotHasBallBecauseBhDisabled)
{
    gameData.team[TeamID::A][RobotID::r1].setPosition(Position2D(-3.0, -5.0,  0.5 * M_PI));
    gameData.team[TeamID::A][RobotID::r1].setBallHandlingModulePresent();
    gameData.team[TeamID::A][RobotID::r1].disableBallHandlers();
    gameData.ball.setLocation(Point2D(-3.0, -4.8));

    gameData.recalculateWorld(SIMULATION_PERIOD_FOR_TEST);

    EXPECT_FALSE(gameData.anyRobotHasBall());
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
    TRACE_FUNCTION("");

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

    EXPECT_FALSE(gameData.team[TeamID::A][RobotID::r1].hasBall());
}


TEST_F(OneMovingRobot, BallAndRobotCollide)
{
    TRACE_FUNCTION("");

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

    EXPECT_FALSE(gameData.team[TeamID::A][RobotID::r1].hasBall());
}


TEST_F(OneMovingRobot, RobotGrabsIncomingBall)
{
    TRACE_FUNCTION("");

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

    EXPECT_TRUE(gameData.team[TeamID::A][RobotID::r1].hasBall());
}


TEST_F(OneMovingRobot, RobotOvertakesBallAndHitsIt)
{
    TRACE_FUNCTION("");

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

    EXPECT_FALSE(gameData.team[TeamID::A][RobotID::r1].hasBall());
}


TEST_F(OneMovingRobot, RobotOvertakesBallAndGrabsIt)
{
    TRACE_FUNCTION("");

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

    EXPECT_TRUE(gameData.team[TeamID::A][RobotID::r1].hasBall());
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
    TRACE_FUNCTION("");

    gameData.team[TeamID::A][RobotID::r1].setVelocity(Velocity2D(0.0, 0.0, 3.0));
    gameData.recalculateWorld(SIMULATION_PERIOD_FOR_TEST);

    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().x);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().y);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().z);

    EXPECT_NEAR(1.0, gameData.team[TeamID::A][RobotID::r1].getPosition().x, 0.001);
    EXPECT_NEAR(1.0, gameData.team[TeamID::A][RobotID::r1].getPosition().y, 0.001);
    EXPECT_NEAR(0.15, gameData.team[TeamID::A][RobotID::r1].getPosition().phi, 0.1);

    EXPECT_TRUE(gameData.team[TeamID::A][RobotID::r1].hasBall());
}


TEST_F(OneRobotWithBall, KicksTheBall)
{
    TRACE_FUNCTION("");

    gameData.team[TeamID::A][RobotID::r1].setKickerSpeed(100.0, 1.0);
    gameData.recalculateWorld(SIMULATION_PERIOD_FOR_TEST);

    EXPECT_FLOAT_EQ(100.0, gameData.ball.getVelocity().x);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().y);
    EXPECT_FLOAT_EQ(0.0, gameData.ball.getVelocity().z);

    EXPECT_NEAR(1.0, gameData.team[TeamID::A][RobotID::r1].getPosition().x, 0.001);
    EXPECT_NEAR(1.0, gameData.team[TeamID::A][RobotID::r1].getPosition().y, 0.001);

    EXPECT_FALSE(gameData.team[TeamID::A][RobotID::r1].hasBall());
}


int main(int argc, char **argv){
    INIT_TRACE("simworldGameDataTest");
    testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    WRITE_TRACE;
    return result;
}
