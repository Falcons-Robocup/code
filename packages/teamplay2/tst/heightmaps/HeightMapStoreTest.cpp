// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * HeightMapStoreTest.cpp
 *
 *  Created on: Nov 9, 2017
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "../TeamplayTest.hpp"

/* SUT */
#include "int/stores/HeightMapStore.hpp"

/* SUT dependencies */
#include "int/stores/BallStore.hpp"
#include "int/stores/ConfigurationStore.hpp"
#include "int/stores/ObstacleStore.hpp"
#include "int/stores/RobotStore.hpp"


/* Testing the heightmap store */

class HeightMapStoreTest : public TeamplayTest
{
public:
    HeightMapStoreTest()
    {
    }
};

TEST_F(HeightMapStoreTest, getDescriptions)
{
    auto descriptions = HeightMapStore::getInstance().getDescriptions();
    EXPECT_THAT(descriptions, Contains(StrEq("Close to own position")));
    EXPECT_THAT(descriptions, Contains(StrEq("In front of opponent goal")));
}


class dataset1 : public HeightMapStoreTest
{
public:
    dataset1()
    {
        /* Set ball position */
        BallStore::getBall().setPosition(Point3D(-3.538, -5.575, 0.000));

        /* Set own Robot position and teammate positions */
        RobotStore::getInstance().clear();
        RobotStore::getInstance().addOwnRobot(Robot(4, RoleEnum::ATTACKER_MAIN, geometry::Pose2D(-2.676, 3.054, 4.618), geometry::Velocity2D()));
        RobotStore::getInstance().addTeammate(Robot(2, RoleEnum::DEFENDER_MAIN, geometry::Pose2D(-2.493, -6.998, 1.559), geometry::Velocity2D()));
        RobotStore::getInstance().addTeammate(Robot(5, RoleEnum::DEFENDER_ASSIST, geometry::Pose2D(0.711, -2.998, 3.785), geometry::Velocity2D()));
        RobotStore::getInstance().addTeammate(Robot(1, RoleEnum::GOALKEEPER, geometry::Pose2D(-0.764, -8.583, 1.560), geometry::Velocity2D()));

        /* Set Obstacle positions */
        ObstacleStore::getInstance().clear();
        ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-3.6,  3.7, 0.0)));
        ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-1.0,  0.5, 0.0)));
        ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D( 1.2, -2.0, 0.0)));
        ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-2.8, -3.0, 0.0)));
    }

    void SetUp()
    {
    }
};


class dataset2 : public HeightMapStoreTest
{
public:
    dataset2()
    {
        /* Set ball position */
        BallStore::getBall().setPosition(Point3D(-2.748, 0.107, 0.000));

        /* Set own Robot position and teammate positions */
        RobotStore::getInstance().clear();
        RobotStore::getInstance().addOwnRobot(Robot(5, RoleEnum::ATTACKER_MAIN, geometry::Pose2D(-1.959, 2.361, 4.332), geometry::Velocity2D()));
        RobotStore::getInstance().addTeammate(Robot(2, RoleEnum::ATTACKER_ASSIST, geometry::Pose2D(3.203, 4.574, 3.814), geometry::Velocity2D()));
        RobotStore::getInstance().addTeammate(Robot(3, RoleEnum::DEFENDER_ASSIST, geometry::Pose2D(-3.116, -6.253, 1.498), geometry::Velocity2D()));
        RobotStore::getInstance().addTeammate(Robot(4, RoleEnum::DEFENDER_MAIN, geometry::Pose2D(-2.992, -0.985, 1.302), geometry::Velocity2D()));

        /* Set Obstacle positions */
        ObstacleStore::getInstance().clear();
        ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D( 2.2,  5.5, 0.0)));
        ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D( 0.7,  4.7, 0.0)));
        ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-1.0,  3.2, 0.0)));
        ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-2.0,  2.2, 0.0)));
    }

    void SetUp()
    {
    }
};


class dataset3 : public HeightMapStoreTest
{
public:
    dataset3()
    {
        /* Set ball position */
        BallStore::getBall().setPosition(Point3D(-3.487, 1.648, 0.000));

        /* Set own Robot position and teammate positions */
        RobotStore::getInstance().clear();
        RobotStore::getInstance().addOwnRobot(Robot(4, RoleEnum::ATTACKER_MAIN, geometry::Pose2D(-1.896, 2.641, 3.646), geometry::Velocity2D()));
        RobotStore::getInstance().addTeammate(Robot(5, RoleEnum::ATTACKER_ASSIST, geometry::Pose2D(3.204, 2.777, 3.286), geometry::Velocity2D()));
        RobotStore::getInstance().addTeammate(Robot(2, RoleEnum::DEFENDER_ASSIST, geometry::Pose2D(-3.290, -6.346, 1.573), geometry::Velocity2D()));
        RobotStore::getInstance().addTeammate(Robot(3, RoleEnum::DEFENDER_MAIN, geometry::Pose2D(-3.797, 0.791, 1.258), geometry::Velocity2D()));

        /* Set Obstacle positions */
        ObstacleStore::getInstance().clear();
        ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-2.0,  4.1, 0.0)));
        ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-0.1,  4.1, 0.0)));
    }

    void SetUp()
    {
    }
};


class dataset4 : public HeightMapStoreTest
{
public:
    dataset4()
    {
        /* Set ball position */
        BallStore::getBall().setPosition(Point3D(1.656, 4.662, 0.000));

        /* Set own Robot position and teammate positions */
        RobotStore::getInstance().clear();
        RobotStore::getInstance().addOwnRobot(Robot(2, RoleEnum::ATTACKER_MAIN, geometry::Pose2D(0.527, 1.654, 0.036), geometry::Velocity2D()));
        RobotStore::getInstance().addTeammate(Robot(3, RoleEnum::ATTACKER_ASSIST, geometry::Pose2D(-3.080, 3.700, 0.187), geometry::Velocity2D()));
        RobotStore::getInstance().addTeammate(Robot(5, RoleEnum::DEFENDER_ASSIST, geometry::Pose2D(1.059, -4.670, 1.715), geometry::Velocity2D()));
        RobotStore::getInstance().addTeammate(Robot(4, RoleEnum::DEFENDER_MAIN, geometry::Pose2D(1.873, 3.139, 1.794), geometry::Velocity2D()));

        /* Set Obstacle positions */
        ObstacleStore::getInstance().clear();
        ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D( 1.6,  6.5, 0.0)));
        ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D( 0.2,  5.9, 0.0)));
        ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D( 0.0,  4.5, 0.0)));
        ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-2.0,  3.6, 0.0)));
    }

    void SetUp()
    {
    }
};


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
