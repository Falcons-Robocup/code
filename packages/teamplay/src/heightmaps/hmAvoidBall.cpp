// Copyright 2018-2020 Ivo Matthijssen (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmAvoidBall.cpp
 *
 *  Created on: Jun 05, 2018
 *      Author: Ivo Matthijssen
 */

#include "int/heightmaps/hmAvoidBall.hpp"

#include "falconsCommon.hpp"
#include "int/stores/ballStore.hpp"
#include "tracing.hpp"

using namespace teamplay;

const static float MIN_DISTANCE_FROM_BALL = 3.0; //change hardcoded 3.0 into input parameter via action in trees

hmAvoidBall::hmAvoidBall() { }

hmAvoidBall::~hmAvoidBall() { }

void hmAvoidBall::precalculate()
{
    // get ball location
    auto ballPos = ballStore::getBall().getLocation();

    for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
    {
        for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
        {
            auto distanceToBall = calc_distance(ballPos, _heightMap(i, j)._center);

            if (distanceToBall < MIN_DISTANCE_FROM_BALL)
            {
            	_heightMap(i, j).setValue(heightMapValues::MIN);
            }
            else
            {
            	_heightMap(i, j).setValue(heightMapValues::NEUTRAL);
            }
        }
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
