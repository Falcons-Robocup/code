// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmBetweenPoiAndClosestObstacleTest.cpp
 *
 *  Created on: Jun 15, 2018
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "../TeamplayTest.hpp"

/* SUT */
#include "int/heightmaps/hmBetweenPoiAndClosestObstacle.hpp"

/* SUT dependencies */
#include "int/stores/BallStore.hpp"
#include "int/stores/ObstacleStore.hpp"
#include "int/stores/RobotStore.hpp"


/* Testing the 'between POI and closest obstacle' heightmap */

class hmBetweenPoiAndClosestObstacleTest : public TeamplayTest
{
public:
    hmBetweenPoiAndClosestObstacleTest()
    {
        BallStore::getBall().reset();
        BallStore::getBall().setPosition(Point3D(1.0, 1.0, 0.0));

        ObstacleStore::getInstance().clear();
        ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D( 3.0, -1.0, 0.0)));
        ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-2.0,  1.0, 0.0)));
        ObstacleStore::getInstance().addObstacle(Obstacle(geometry::Pose2D(-2.0,  1.2, 0.0)));

        RobotStore::getInstance().clear();
        RobotStore::getInstance().addOwnRobot(Robot());
    }
    hmBetweenPoiAndClosestObstacle _hmBetweenPoiAndClosestObstacle;
};

TEST_F(hmBetweenPoiAndClosestObstacleTest, poiBall)
{
    RobotStore::getInstance().setOwnRobotRole(RoleEnum::ATTACKER_MAIN);
    _hmBetweenPoiAndClosestObstacle.refine();
}

TEST_F(hmBetweenPoiAndClosestObstacleTest, poiOwnGoal)
{
    RobotStore::getInstance().setOwnRobotRole(RoleEnum::DEFENDER_MAIN);
    _hmBetweenPoiAndClosestObstacle.refine();
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
