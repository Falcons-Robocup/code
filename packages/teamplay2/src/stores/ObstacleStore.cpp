// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ObstacleStore.cpp
 *
 *  Created on: Jan 5, 2018
 *      Author: Coen Tempelaars
 */

#include <algorithm>

#include "int/stores/ObstacleStore.hpp"
#include "tracing.hpp"

using namespace teamplay;


/* Implementation of class ObstacleStore */
ObstacleStore::ObstacleStore() { }

ObstacleStore::~ObstacleStore() { }

void ObstacleStore::clear()
{
    _all_obstacles.clear();
}

void ObstacleStore::addObstacle(const Obstacle& o)
{
    _all_obstacles.push_back(o);
}

std::vector<Obstacle> ObstacleStore::getAllObstacles() const
{
    return _all_obstacles;
}

std::vector<Obstacle> ObstacleStore::getAllObstaclesSortedByDistanceTo(const Point2D& p) const
{
    std::vector<Obstacle> retVal = _all_obstacles;

    std::sort(retVal.begin(), retVal.end(),
              [&](const Obstacle& first, const Obstacle& second)
              { return first.getDistanceTo(p) < second.getDistanceTo(p); });

    return retVal;
}
