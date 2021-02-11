// Copyright 2018-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmBetweenPoiAndClosestObstacleTest.cpp
 *
 *  Created on: Jun 15, 2018
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "teamplayTest.hpp"

/* SUT */
#include "int/heightmaps/hmBetweenPoiAndClosestObstacle.hpp"

/* SUT dependencies */
#include "int/stores/ballStore.hpp"
#include "int/stores/obstacleStore.hpp"
#include "int/stores/robotStore.hpp"


/* Testing the 'between POI and closest obstacle' heightmap */

class hmBetweenPoiAndClosestObstacleTest : public TeamplayTest
{
public:
    hmBetweenPoiAndClosestObstacleTest()
    {
        ballStore::getBall().reset();
        ballStore::getBall().setPosition(Point3D(1.0, 1.0, 0.0));

        obstacleStore::getInstance().clear();
        obstacleStore::getInstance().addObstacle(obstacle(Position2D( 3.0, -1.0, 0.0)));
        obstacleStore::getInstance().addObstacle(obstacle(Position2D(-2.0,  1.0, 0.0)));
        obstacleStore::getInstance().addObstacle(obstacle(Position2D(-2.0,  1.2, 0.0)));

        robotStore::getInstance().clear();
        robotStore::getInstance().addOwnRobot(robot());
    }
    hmBetweenPoiAndClosestObstacle _hmBetweenPoiAndClosestObstacle;
    parameterMap_t _parameters;
};

TEST_F(hmBetweenPoiAndClosestObstacleTest, poiBall)
{
    _parameters["POI"] = "ball";

    _hmBetweenPoiAndClosestObstacle.refine(_parameters);
}

TEST_F(hmBetweenPoiAndClosestObstacleTest, poiOwnGoal)
{
    _parameters["POI"] = "P_OWN_GOALLINE_CENTER";

    _hmBetweenPoiAndClosestObstacle.refine(_parameters);
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
