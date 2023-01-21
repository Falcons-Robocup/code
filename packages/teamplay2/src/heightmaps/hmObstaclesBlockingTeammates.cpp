// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmObstaclesBlockingTeammates.cpp
 *
 *  Created on: Jun 15, 2018
 *      Author: Coen Tempelaars
 */

#include "int/heightmaps/hmObstaclesBlockingTeammates.hpp"

#include <cmath>
#include <limits>

#include "falconsCommon.hpp"
#include "int/stores/ObstacleStore.hpp"
#include "int/stores/RobotStore.hpp"
#include "tracing.hpp"

using namespace teamplay;


const static float LOW  = 2.8;
const static float HIGH = 2.6;

hmObstaclesBlockingTeammates::hmObstaclesBlockingTeammates() { }

hmObstaclesBlockingTeammates::~hmObstaclesBlockingTeammates() { }

void hmObstaclesBlockingTeammates::precalculate()
{
    if (!_heightmapPrecalculated)
    {
        auto low = LOW;
        auto high = HIGH;
        auto alpha = 100 / (high - low);

        auto obstacles = ObstacleStore::getInstance().getAllObstacles();
        auto teammates = RobotStore::getInstance().getAllRobotsExclOwnRobotInArea(FieldArea::OPP_SIDE);

        for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
        {
            for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
            {
                double minValue = std::numeric_limits<double>::max();
                for (auto teammate = teammates.begin(); teammate != teammates.end(); teammate++)
                {
                    auto teammateLocation = teammate->getLocation();
                    for (auto obstacle = obstacles.begin(); obstacle != obstacles.end(); obstacle++)
                    {
                        auto obstacleLocation = obstacle->getLocation();
                        auto angle = angleOnPath(_heightMap(i, j)._center, obstacleLocation, teammateLocation);
                        auto value = alpha * (angle - low);

                        if (value < minValue)
                        {
                            minValue = value;
                        }
                    }
                }
                _heightMap(i, j).setValue(minValue, HeightMapValues::MIN, HeightMapValues::NEUTRAL);
            }
        }
        _heightmapPrecalculated = true;
    }

    return;
}

std::string hmObstaclesBlockingTeammates::getDescription() const
{
    return "Avoid obstacles that block teammates on opponent half";
}

std::string hmObstaclesBlockingTeammates::getFilename() const
{
    return "hmObstaclesBlockingTeammates";
}
