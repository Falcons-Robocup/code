// Copyright 2018 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * obstacleStoreTest.cpp
 *
 *  Created on: Jan 5, 2018
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "teamplayTest.hpp"

/* SUT */
#include "int/stores/obstacleStore.hpp"

/* SUT dependencies */



class obstacleStoreTest : public TeamplayTest
{
public:
    obstacleStoreTest()
    {
        obstacleStore::getInstance().clear();

        obstacle obs1(Position2D(3.5, -2.5, 0.0));
        obstacle obs2(Position2D(-1.2, -2.3, -3.4), Velocity2D(1.2, 2.3, 3.4));

        obstacleStore::getInstance().addObstacle(obs1);
        obstacleStore::getInstance().addObstacle(obs2);
    }
};

TEST_F(obstacleStoreTest, getAllObstacles)
{
    auto obstacles = obstacleStore::getInstance().getAllObstacles();
    EXPECT_EQ(2, obstacles.size());
    EXPECT_EQ(Point2D( 3.5, -2.5), obstacles.at(0).getLocation());
    EXPECT_EQ(Point2D(-1.2, -2.3), obstacles.at(1).getLocation());
}

TEST_F(obstacleStoreTest, getAllObstaclesSortedByDistanceTo)
{
    auto obstacles = obstacleStore::getInstance().getAllObstaclesSortedByDistanceTo(Point2D(-1.5, -2.5));
    EXPECT_EQ(2, obstacles.size());
    EXPECT_EQ(Point2D(-1.2, -2.3), obstacles.at(0).getLocation());
    EXPECT_EQ(Point2D( 3.5, -2.5), obstacles.at(1).getLocation());
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
