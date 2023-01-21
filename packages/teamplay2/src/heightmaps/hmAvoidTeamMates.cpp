// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmAvoidTeamMates.cpp
 *
 *  Created on: Nov 16, 2017
 *      Author: Coen Tempelaars
 */

#include "int/heightmaps/hmAvoidTeamMates.hpp"

#include "falconsCommon.hpp"
#include "int/stores/RobotStore.hpp"
#include "tracing.hpp"

using namespace teamplay;


const static float LOW  = 7.0;
const static float HIGH = 13.25;

hmAvoidTeamMates::hmAvoidTeamMates() { }

hmAvoidTeamMates::~hmAvoidTeamMates() { }

void hmAvoidTeamMates::precalculate()
{
    if (!_heightmapPrecalculated)
    {
        auto low = LOW;
        auto high = HIGH;
        auto alpha = 100 / (high - low);

        auto teammembers = RobotStore::getInstance().getAllRobotsExclOwnRobot();

        for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
        {
            for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
            {
                double minDistance = std::numeric_limits<double>::max();
                for (auto teammember = teammembers.begin(); teammember != teammembers.end(); teammember++)
                {
                    auto distance = calc_distance(teammember->getLocation(), _heightMap(i, j)._center);

                    if (distance < minDistance)
                    {
                        minDistance = distance;
                    }
                }
                _heightMap(i, j).setValue(alpha * (minDistance - low), HeightMapValues::MIN, HeightMapValues::NEUTRAL);
            }
        }
        _heightmapPrecalculated = true;
    }

    return;
}

std::string hmAvoidTeamMates::getDescription() const
{
    return "Avoid team mates";
}

std::string hmAvoidTeamMates::getFilename() const
{
    return "hmAvoidTeamMates";
}
