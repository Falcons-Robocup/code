// Copyright 2017-2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * heightMapStoreTest.cpp
 *
 *  Created on: Nov 9, 2017
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "teamplayTest.hpp"

/* SUT */
#include "int/stores/heightMapStore.hpp"

/* SUT dependencies */
#include "int/stores/ballStore.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/stores/obstacleStore.hpp"
#include "int/stores/robotStore.hpp"


/* Testing the heightmap store */

class heightMapStoreTest : public TeamplayTest
{
public:
    heightMapStoreTest()
    {
    }
};

TEST_F(heightMapStoreTest, getDescriptions)
{
    auto descriptions = heightMapStore::getInstance().getDescriptions();
    EXPECT_THAT(descriptions, Contains(StrEq("Close to own position")));
    EXPECT_THAT(descriptions, Contains(StrEq("In front of opponent goal")));
}


class dataset1 : public heightMapStoreTest
{
public:
    dataset1()
    {
        /* Set ball position */
        ballStore::getBall().setPosition(Point3D(-3.538, -5.575, 0.000));

        /* Set own robot position and teammate positions */
        robotStore::getInstance().clear();
        robotStore::getInstance().addOwnRobot(robot(4, treeEnum::ATTACKER_MAIN, Position2D(-2.676, 3.054, 4.618), Velocity2D()));
        robotStore::getInstance().addTeammate(robot(2, treeEnum::DEFENDER_MAIN, Position2D(-2.493, -6.998, 1.559), Velocity2D()));
        robotStore::getInstance().addTeammate(robot(5, treeEnum::DEFENDER_ASSIST, Position2D(0.711, -2.998, 3.785), Velocity2D()));
        robotStore::getInstance().addTeammate(robot(1, treeEnum::R_GOALKEEPER, Position2D(-0.764, -8.583, 1.560), Velocity2D()));

        /* Set obstacle positions */
        obstacleStore::getInstance().clear();
        obstacleStore::getInstance().addObstacle(obstacle(Position2D(-3.6,  3.7, 0.0)));
        obstacleStore::getInstance().addObstacle(obstacle(Position2D(-1.0,  0.5, 0.0)));
        obstacleStore::getInstance().addObstacle(obstacle(Position2D( 1.2, -2.0, 0.0)));
        obstacleStore::getInstance().addObstacle(obstacle(Position2D(-2.8, -3.0, 0.0)));
    }

    void SetUp()
    {
        heightMapStore::getInstance().precalculateAll();
    }
};


class dataset2 : public heightMapStoreTest
{
public:
    dataset2()
    {
        /* Set ball position */
        ballStore::getBall().setPosition(Point3D(-2.748, 0.107, 0.000));

        /* Set own robot position and teammate positions */
        robotStore::getInstance().clear();
        robotStore::getInstance().addOwnRobot(robot(5, treeEnum::ATTACKER_MAIN, Position2D(-1.959, 2.361, 4.332), Velocity2D()));
        robotStore::getInstance().addTeammate(robot(2, treeEnum::ATTACKER_ASSIST, Position2D(3.203, 4.574, 3.814), Velocity2D()));
        robotStore::getInstance().addTeammate(robot(3, treeEnum::DEFENDER_ASSIST, Position2D(-3.116, -6.253, 1.498), Velocity2D()));
        robotStore::getInstance().addTeammate(robot(4, treeEnum::DEFENDER_MAIN, Position2D(-2.992, -0.985, 1.302), Velocity2D()));

        /* Set obstacle positions */
        obstacleStore::getInstance().clear();
        obstacleStore::getInstance().addObstacle(obstacle(Position2D( 2.2,  5.5, 0.0)));
        obstacleStore::getInstance().addObstacle(obstacle(Position2D( 0.7,  4.7, 0.0)));
        obstacleStore::getInstance().addObstacle(obstacle(Position2D(-1.0,  3.2, 0.0)));
        obstacleStore::getInstance().addObstacle(obstacle(Position2D(-2.0,  2.2, 0.0)));
    }

    void SetUp()
    {
        heightMapStore::getInstance().precalculateAll();
    }
};


class dataset3 : public heightMapStoreTest
{
public:
    dataset3()
    {
        /* Set ball position */
        ballStore::getBall().setPosition(Point3D(-3.487, 1.648, 0.000));

        /* Set own robot position and teammate positions */
        robotStore::getInstance().clear();
        robotStore::getInstance().addOwnRobot(robot(4, treeEnum::ATTACKER_MAIN, Position2D(-1.896, 2.641, 3.646), Velocity2D()));
        robotStore::getInstance().addTeammate(robot(5, treeEnum::ATTACKER_ASSIST, Position2D(3.204, 2.777, 3.286), Velocity2D()));
        robotStore::getInstance().addTeammate(robot(2, treeEnum::DEFENDER_ASSIST, Position2D(-3.290, -6.346, 1.573), Velocity2D()));
        robotStore::getInstance().addTeammate(robot(3, treeEnum::DEFENDER_MAIN, Position2D(-3.797, 0.791, 1.258), Velocity2D()));

        /* Set obstacle positions */
        obstacleStore::getInstance().clear();
        obstacleStore::getInstance().addObstacle(obstacle(Position2D(-2.0,  4.1, 0.0)));
        obstacleStore::getInstance().addObstacle(obstacle(Position2D(-0.1,  4.1, 0.0)));
    }

    void SetUp()
    {
        heightMapStore::getInstance().precalculateAll();
    }
};


class dataset4 : public heightMapStoreTest
{
public:
    dataset4()
    {
        /* Set ball position */
        ballStore::getBall().setPosition(Point3D(1.656, 4.662, 0.000));

        /* Set own robot position and teammate positions */
        robotStore::getInstance().clear();
        robotStore::getInstance().addOwnRobot(robot(2, treeEnum::ATTACKER_MAIN, Position2D(0.527, 1.654, 0.036), Velocity2D()));
        robotStore::getInstance().addTeammate(robot(3, treeEnum::ATTACKER_ASSIST, Position2D(-3.080, 3.700, 0.187), Velocity2D()));
        robotStore::getInstance().addTeammate(robot(5, treeEnum::DEFENDER_ASSIST, Position2D(1.059, -4.670, 1.715), Velocity2D()));
        robotStore::getInstance().addTeammate(robot(4, treeEnum::DEFENDER_MAIN, Position2D(1.873, 3.139, 1.794), Velocity2D()));

        /* Set obstacle positions */
        obstacleStore::getInstance().clear();
        obstacleStore::getInstance().addObstacle(obstacle(Position2D( 1.6,  6.5, 0.0)));
        obstacleStore::getInstance().addObstacle(obstacle(Position2D( 0.2,  5.9, 0.0)));
        obstacleStore::getInstance().addObstacle(obstacle(Position2D( 0.0,  4.5, 0.0)));
        obstacleStore::getInstance().addObstacle(obstacle(Position2D(-2.0,  3.6, 0.0)));
    }

    void SetUp()
    {
        heightMapStore::getInstance().precalculateAll();
    }
};


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
