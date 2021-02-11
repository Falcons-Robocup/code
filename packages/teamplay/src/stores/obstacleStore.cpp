// Copyright 2018-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * obstacleStore.cpp
 *
 *  Created on: Jan 5, 2018
 *      Author: Coen Tempelaars
 */

#include <algorithm>

#include "int/stores/obstacleStore.hpp"
#include "tracing.hpp"

using namespace teamplay;


/* Implementation of class obstacleStore */
obstacleStore::obstacleStore() { }

obstacleStore::~obstacleStore() { }

void obstacleStore::clear()
{
    _all_obstacles.clear();
}

void obstacleStore::addObstacle(const obstacle& o)
{
    _all_obstacles.push_back(o);
}

std::vector<obstacle> obstacleStore::getAllObstacles() const
{
    return _all_obstacles;
}

std::vector<obstacle> obstacleStore::getAllObstaclesSortedByDistanceTo(const Point2D& p) const
{
    std::vector<obstacle> retVal = _all_obstacles;

    std::sort(retVal.begin(), retVal.end(),
              [&](const obstacle& first, const obstacle& second)
              { return first.getDistanceTo(p) < second.getDistanceTo(p); });

    return retVal;
}
