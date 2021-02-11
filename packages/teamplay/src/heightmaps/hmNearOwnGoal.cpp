// Copyright 2018-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmNearOwnGoal.cpp
 *
 *  Created on: Jun 10, 2018
 *      Author: Coen Tempelaars
 */

#include "int/heightmaps/hmNearOwnGoal.hpp"

#include "falconsCommon.hpp"
#include "int/stores/fieldDimensionsStore.hpp"
#include "tracing.hpp"

using namespace teamplay;


const static float LOW  = 14.0;
const static float HIGH = 1.0;

hmNearOwnGoal::hmNearOwnGoal() { }

hmNearOwnGoal::~hmNearOwnGoal() { }

void hmNearOwnGoal::precalculate()
{
    auto low = LOW;
    auto high = HIGH;
    auto alpha = 100 / (high - low);

    auto yLimit = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OWN_GOALLINE_CENTER).y / 4;
    auto gravityPoint = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::CENTER);

    for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
    {
        for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
        {
            if (_heightMap(i, j)._center.y < (yLimit))
            {
                auto distanceToGravityPoint = calc_distance(gravityPoint, _heightMap(i, j)._center);

                _heightMap(i, j).setValue(alpha * (distanceToGravityPoint - low), heightMapValues::NEUTRAL, heightMapValues::MAX);
            }
        }
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
