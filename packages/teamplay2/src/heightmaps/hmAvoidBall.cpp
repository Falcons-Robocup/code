// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmAvoidBall.cpp
 *
 *  Created on: Jun 05, 2018
 *      Author: Ivo Matthijssen
 */

#include "int/heightmaps/hmAvoidBall.hpp"

#include "falconsCommon.hpp"
#include "int/stores/BallStore.hpp"
#include "tracing.hpp"

using namespace teamplay;

const static float MIN_DISTANCE_FROM_BALL = 3.0; //change hardcoded 3.0 into input parameter via action in trees

hmAvoidBall::hmAvoidBall() { }

hmAvoidBall::~hmAvoidBall() { }

void hmAvoidBall::precalculate()
{
    if (!_heightmapPrecalculated)
    {
        // get ball location
        auto ballPos = BallStore::getBall().getLocation();

        for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
        {
            for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
            {
                auto distanceToBall = calc_distance(ballPos, _heightMap(i, j)._center);

                if (distanceToBall < MIN_DISTANCE_FROM_BALL)
                {
                    _heightMap(i, j).setValue(HeightMapValues::MIN);
                }
                else
                {
                    _heightMap(i, j).setValue(HeightMapValues::NEUTRAL);
                }
            }
        }
        _heightmapPrecalculated = true;
    }

    return;
}

std::string hmAvoidBall::getDescription() const
{
    return "Avoid ball";
}

std::string hmAvoidBall::getFilename() const
{
    return "hmAvoidBall";
}
