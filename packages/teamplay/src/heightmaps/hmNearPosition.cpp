// Copyright 2018-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmNearPosition.hpp
 *
 *  Created on: June 5, 2018
 *      Author: Jan Feitsma
 */

#include <boost/lexical_cast.hpp>

#include "int/heightmaps/hmNearPosition.hpp"

#include "falconsCommon.hpp"

#include "tracing.hpp"

using namespace teamplay;


hmNearPosition::hmNearPosition()
{
    reset();
}

hmNearPosition::~hmNearPosition()
{
}

void hmNearPosition::precalculate()
{
    // cannot do anything yet, need parameters
    return;
}

abstractHeightMap hmNearPosition::refine(const parameterMap_t& params)
{
    // configuration
    float alpha = 10.0; // magic number
    // extract parameters
    float positionX = 0.0;
    float positionY = 0.0;
    auto it = params.find("positionX");
    if (it != params.end())
    {
        positionX = boost::lexical_cast<float>(it->second);
    }
    it = params.find("positionY");
    if (it != params.end())
    {
        positionY = boost::lexical_cast<float>(it->second);
    }
    // calculate the height map
    for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
    {
        for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
        {
            auto distance = calc_distance(Point2D(positionX, positionY), _heightMap(i, j)._center);
            _heightMap(i, j).setValue(100.0 - alpha * distance);
        }
    }
    return *this;
}

std::string hmNearPosition::getDescription() const
{
    return "Near position";
}

std::string hmNearPosition::getFilename() const
{
    return "hmNearPosition";
}

