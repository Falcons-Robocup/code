// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cWorldStateFunctions.hpp
 *
 *  Created on: Sep 18, 2015
 *      Author: Ivo Matthijssen
 */

#ifndef WORLDSTATEFUNCTIONS_HPP_
#define WORLDSTATEFUNCTIONS_HPP_


#include <vector>

#include "int/types/Obstacle.hpp"


namespace teamplay
{

class WorldStateFunctions
{
public:
    static WorldStateFunctions& getInstance()
    {
        static WorldStateFunctions instance; // Guaranteed to be destroyed.
                                                // Instantiated on first use.
        return instance;
    }

    void getObstructingObstaclesInPath(const Point2D robotPos, const Point2D targetPos, const float radiusObjectMeters, std::vector<Obstacle> &obstacles);


private:
    WorldStateFunctions();
    WorldStateFunctions(WorldStateFunctions const&) = delete;
    WorldStateFunctions(WorldStateFunctions &&) = delete;
    WorldStateFunctions operator=(WorldStateFunctions const&) = delete;
    WorldStateFunctions operator=(WorldStateFunctions &&) = delete;
    ~WorldStateFunctions() = default;

};

}

#endif // WORLDSTATEFUNCTIONS_HPP_
