// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ObstacleStoreTest.cpp
 *
 *  Created on: Jan 5, 2018
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "../TeamplayTest.hpp"

/* SUT */
#include "int/stores/ObstacleStore.hpp"

/* SUT dependencies */



class ObstacleStoreTest : public TeamplayTest
{
public:
    ObstacleStoreTest()
    {
        ObstacleStore::getInstance().clear();

        Obstacle obs1(geometry::Pose2D(3.5, -2.5, 0.0));
        Obstacle obs2(geometry::Pose2D(-1.2, -2.3, -3.4), geometry::Velocity2D(1.2, 2.3, 3.4));

        ObstacleStore::getInstance().addObstacle(obs1);
        ObstacleStore::getInstance().addObstacle(obs2);
    }
};

TEST_F(ObstacleStoreTest, getAllObstacles)
{
    auto obstacles = ObstacleStore::getInstance().getAllObstacles();
    EXPECT_EQ(2, obstacles.size());
    EXPECT_EQ(Point2D( 3.5, -2.5), obstacles.at(0).getLocation());
    EXPECT_EQ(Point2D(-1.2, -2.3), obstacles.at(1).getLocation());
}

TEST_F(ObstacleStoreTest, getAllObstaclesSortedByDistanceTo)
{
    auto obstacles = ObstacleStore::getInstance().getAllObstaclesSortedByDistanceTo(Point2D(-1.5, -2.5));
    EXPECT_EQ(2, obstacles.size());
    EXPECT_EQ(Point2D(-1.2, -2.3), obstacles.at(0).getLocation());
    EXPECT_EQ(Point2D( 3.5, -2.5), obstacles.at(1).getLocation());
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
