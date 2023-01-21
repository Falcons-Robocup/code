// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmObstaclesBlockingOppGoal.cpp
 *
 *  Created on: Jan 6, 2018
 *      Author: Coen Tempelaars
 */

#include "int/heightmaps/hmObstaclesBlockingOppGoal.hpp"

#include <cmath>
#include <limits>

#include "falconsCommon.hpp"
#include "int/stores/FieldDimensionsStore.hpp"
#include "int/stores/ObstacleStore.hpp"
#include "tracing.hpp"

using namespace teamplay;


const static float LOW  = 0.417;
const static float HIGH = 0.734;

const static float DISTANCE_BEHIND_OBSTACLE = 1.0;

hmObstaclesBlockingOppGoal::hmObstaclesBlockingOppGoal() { }

hmObstaclesBlockingOppGoal::~hmObstaclesBlockingOppGoal() { }

void hmObstaclesBlockingOppGoal::precalculate()
{
    if (!_heightmapPrecalculated)
    {
        auto low = LOW;
        auto high = HIGH;
        auto alpha = 100 / (high - low);

        auto oppGoal = FieldDimensionsStore::getFieldDimensions().getLocation(FieldPOI::OPP_GOALLINE_CENTER);
        auto obstacles = ObstacleStore::getInstance().getAllObstacles();

        for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
        {
            for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
            {
                double minDifference = std::numeric_limits<double>::max();
                for (auto obstacle = obstacles.begin(); obstacle != obstacles.end(); obstacle++)
                {
                    auto obstacleLocation = obstacle->getLocation();
                    auto refinedObstacleLocation = (1 - (DISTANCE_BEHIND_OBSTACLE / (obstacleLocation - oppGoal).size())) * (obstacleLocation - oppGoal) + oppGoal;

                    auto oppToGoalAngle = angle_between_two_points_0_2pi(refinedObstacleLocation.x, refinedObstacleLocation.y, oppGoal.x, oppGoal.y);
                    auto heightMapFieldToOppAngle = angle_between_two_points_0_2pi(_heightMap(i, j)._center.x, _heightMap(i, j)._center.y, refinedObstacleLocation.x, refinedObstacleLocation.y);
                    auto absDifference = std::abs(oppToGoalAngle - heightMapFieldToOppAngle);

                    if (absDifference < minDifference)
                    {
                        minDifference = absDifference;
                    }
                }
                _heightMap(i, j).setValue(alpha * (minDifference - low), HeightMapValues::MIN, HeightMapValues::NEUTRAL);
            }
        }
        _heightmapPrecalculated = true;
    }

    return;
}

std::string hmObstaclesBlockingOppGoal::getDescription() const
{
    return "Avoid obstacles that block the opponent goal";
}

std::string hmObstaclesBlockingOppGoal::getFilename() const
{
    return "hmObstaclesBlockingOppGoal";
}
