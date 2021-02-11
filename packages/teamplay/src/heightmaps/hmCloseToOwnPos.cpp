// Copyright 2017-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmCloseToOwnPos.cpp
 *
 *  Created on: Nov 16, 2017
 *      Author: Coen Tempelaars
 */

#include "int/heightmaps/hmCloseToOwnPos.hpp"

#include "falconsCommon.hpp"
#include "int/stores/robotStore.hpp"
#include "tracing.hpp"

using namespace teamplay;


const static float LOW  = 5.7;
const static float HIGH = 2.0;

hmCloseToOwnPos::hmCloseToOwnPos() { }

hmCloseToOwnPos::~hmCloseToOwnPos() { }

void hmCloseToOwnPos::precalculate()
{
    auto low = LOW;
    auto high = HIGH;
    auto alpha = 100 / (high - low);

    auto ownPos = robotStore::getInstance().getOwnRobot().getLocation();

    for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
    {
        for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
        {
            auto distanceToOwnPos = calc_distance(ownPos, _heightMap(i, j)._center);

            _heightMap(i, j).setValue(alpha * (distanceToOwnPos - low), heightMapValues::NEUTRAL, heightMapValues::MAX);
        }
    }

    return;
}

std::string hmCloseToOwnPos::getDescription() const
{
    return "Close to own position";
}

std::string hmCloseToOwnPos::getFilename() const
{
    return "hmCloseToOwnPos";
}
