// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmNearOwnGoal.cpp
 *
 *  Created on: Jun 10, 2018
 *      Author: Coen Tempelaars
 */

#include "int/heightmaps/hmNearOwnGoal.hpp"

#include "falconsCommon.hpp"
#include "int/stores/FieldDimensionsStore.hpp"
#include "tracing.hpp"

using namespace teamplay;


const static float LOW  = 14.0;
const static float HIGH = 1.0;

hmNearOwnGoal::hmNearOwnGoal() { }

hmNearOwnGoal::~hmNearOwnGoal() { }

void hmNearOwnGoal::precalculate()
{
    if (!_heightmapPrecalculated)
    {
        auto low = LOW;
        auto high = HIGH;
        auto alpha = 100 / (high - low);

        auto yLimit = FieldDimensionsStore::getFieldDimensions().getLocation(FieldPOI::OWN_GOALLINE_CENTER).y / 4;
        auto gravityPoint = FieldDimensionsStore::getFieldDimensions().getLocation(FieldPOI::CENTER);

        for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
        {
            for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
            {
                if (_heightMap(i, j)._center.y < (yLimit))
                {
                    auto distanceToGravityPoint = calc_distance(gravityPoint, _heightMap(i, j)._center);

                    _heightMap(i, j).setValue(alpha * (distanceToGravityPoint - low), HeightMapValues::NEUTRAL, HeightMapValues::MAX);
                }
            }
        }
        _heightmapPrecalculated = true;
    }

    return;
}

std::string hmNearOwnGoal::getDescription() const
{
    return "Near own goal";
}

std::string hmNearOwnGoal::getFilename() const
{
    return "hmNearOwnGoal";
}
