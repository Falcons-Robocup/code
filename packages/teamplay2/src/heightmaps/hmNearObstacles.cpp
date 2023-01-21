// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmNearObstacles.cpp
 *
 *  Created on: Jun 13, 2018
 *      Author: Coen Tempelaars
 */

#include "int/heightmaps/hmNearObstacles.hpp"

#include "falconsCommon.hpp"
#include "int/stores/ObstacleStore.hpp"
#include "tracing.hpp"

using namespace teamplay;


const static float LOW  = 5.0;
const static float HIGH = 1.0;

hmNearObstacles::hmNearObstacles() { }

hmNearObstacles::~hmNearObstacles() { }

void hmNearObstacles::precalculate()
{
    if (!_heightmapPrecalculated)
    {
        auto low = LOW;
        auto high = HIGH;
        auto alpha = 100 / (high - low);

        auto obstacles = ObstacleStore::getInstance().getAllObstacles();

        for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
        {
            for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
            {
                double minDistance = std::numeric_limits<double>::max();
                for (auto obstacle = obstacles.begin(); obstacle != obstacles.end(); obstacle++)
                {
                    auto distance = calc_distance(obstacle->getLocation(), _heightMap(i, j)._center);

                    if (distance < minDistance)
                    {
                        minDistance = distance;
                    }
                }
                _heightMap(i, j).setValue(alpha * (minDistance - low), HeightMapValues::NEUTRAL, HeightMapValues::MAX);
            }
        }
        _heightmapPrecalculated = true;
    }

    return;
}

std::string hmNearObstacles::getDescription() const
{
    return "Near obstacles";
}

std::string hmNearObstacles::getFilename() const
{
    return "hmNearObstacles";
}
