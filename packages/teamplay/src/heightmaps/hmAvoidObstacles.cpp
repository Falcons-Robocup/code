// Copyright 2018-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmAvoidObstacles.cpp
 *
 *  Created on: Apr 28, 2018
 *      Author: Coen Tempelaars
 */

#include "int/heightmaps/hmAvoidObstacles.hpp"

#include "falconsCommon.hpp"
#include "int/stores/obstacleStore.hpp"
#include "tracing.hpp"

using namespace teamplay;


const static float LOW  = 2.8;
const static float HIGH = 5.5;

hmAvoidObstacles::hmAvoidObstacles() { }

hmAvoidObstacles::~hmAvoidObstacles() { }

void hmAvoidObstacles::precalculate()
{
    auto low = LOW;
    auto high = HIGH;
    auto alpha = 100 / (high - low);

    auto obstacles = obstacleStore::getInstance().getAllObstacles();

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
            _heightMap(i, j).setValue(alpha * (minDistance - low), heightMapValues::MIN, heightMapValues::NEUTRAL);
        }
    }

    return;
}

std::string hmAvoidObstacles::getDescription() const
{
    return "Avoid obstacles";
}

std::string hmAvoidObstacles::getFilename() const
{
    return "hmAvoidObstacles";
}
