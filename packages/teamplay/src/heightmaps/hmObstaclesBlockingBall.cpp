// Copyright 2018-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmObstaclesBlockingBall.cpp
 *
 *  Created on: Jan 23, 2018
 *      Author: Coen Tempelaars
 */

#include "int/heightmaps/hmObstaclesBlockingBall.hpp"

#include <cmath>
#include <limits>

#include "falconsCommon.hpp"
#include "int/stores/ballStore.hpp"
#include "int/stores/obstacleStore.hpp"
#include "tracing.hpp"

using namespace teamplay;


const static float LOW  = 2.8;
const static float HIGH = 2.6;

hmObstaclesBlockingBall::hmObstaclesBlockingBall() { }

hmObstaclesBlockingBall::~hmObstaclesBlockingBall() { }

void hmObstaclesBlockingBall::precalculate()
{
    auto low = LOW;
    auto high = HIGH;
    auto alpha = 100 / (high - low);

    auto obstacles = obstacleStore::getInstance().getAllObstacles();
    auto ballLocation = ballStore::getBall().getLocation();

    for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
    {
        for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
        {
            double minValue = std::numeric_limits<double>::max();
            for (auto obstacle = obstacles.begin(); obstacle != obstacles.end(); obstacle++)
            {
                auto obstacleLocation = obstacle->getLocation();
                auto angle = angleOnPath(_heightMap(i, j)._center, obstacleLocation, ballLocation);
                auto value = alpha * (angle - low);

                if (value < minValue)
                {
                    minValue = value;
                }
            }
            _heightMap(i, j).setValue(minValue, heightMapValues::MIN, heightMapValues::NEUTRAL);
        }
    }

    return;
}

std::string hmObstaclesBlockingBall::getDescription() const
{
    return "Avoid obstacles that block the ball";
}

std::string hmObstaclesBlockingBall::getFilename() const
{
    return "hmObstaclesBlockingBall";
}
