// Copyright 2021 Erik Kouters (Falcons)
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
#include "int/stores/BallStore.hpp"
#include "int/stores/ObstacleStore.hpp"
#include "tracing.hpp"

using namespace teamplay;


const static float LOW  = 2.8;
const static float HIGH = 2.6;

hmObstaclesBlockingBall::hmObstaclesBlockingBall() { }

hmObstaclesBlockingBall::~hmObstaclesBlockingBall() { }

void hmObstaclesBlockingBall::precalculate()
{
    if (!_heightmapPrecalculated)
    {
        auto low = LOW;
        auto high = HIGH;
        auto alpha = 100 / (high - low);

        auto obstacles = ObstacleStore::getInstance().getAllObstacles();
        auto ballLocation = BallStore::getBall().getLocation();

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
                _heightMap(i, j).setValue(minValue, HeightMapValues::MIN, HeightMapValues::NEUTRAL);
            }
        }
        _heightmapPrecalculated = true;
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
