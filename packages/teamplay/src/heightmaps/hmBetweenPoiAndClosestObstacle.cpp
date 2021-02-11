// Copyright 2018-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * hmBetweenPoiAndClosestObstacle.cpp
 *
 *  Created on: Jun 15, 2018
 *      Author: Coen Tempelaars
 */

#include "int/heightmaps/hmBetweenPoiAndClosestObstacle.hpp"

#include "falconsCommon.hpp"
#include "polygon2D.hpp"

#include "int/stores/ballStore.hpp"
#include "int/stores/fieldDimensionsStore.hpp"
#include "int/stores/obstacleStore.hpp"
#include "int/stores/robotStore.hpp"
#include "cDiagnostics.hpp"
#include "tracing.hpp"

using namespace teamplay;


const static float WIDTH = 0.3;

hmBetweenPoiAndClosestObstacle::hmBetweenPoiAndClosestObstacle() { }

hmBetweenPoiAndClosestObstacle::~hmBetweenPoiAndClosestObstacle() { }

void hmBetweenPoiAndClosestObstacle::precalculate()
{
    /*
     * Deliberately left empty, as there is nothing to pre-calculate.
     */
    return;
}

abstractHeightMap hmBetweenPoiAndClosestObstacle::refine(const parameterMap_t& params)
{
    auto poiLocation = Point2D();
    auto it = params.find("POI");
    if (it != params.end())
    {
        if (it->second == "ball")
        {
            TRACE("hmBetweenPoiAndClosestObstacle: POI: ball");
            poiLocation = ballStore::getBall().getLocation();
        }
        else if (it->second == "P_OWN_GOALLINE_CENTER")
        {
            TRACE("hmBetweenPoiAndClosestObstacle: POI: P_OWN_GOALLINE_CENTER");
            poiLocation = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OWN_GOALLINE_CENTER);
        }
        else
        {
            TRACE_ERROR("Parameter 'POI' has an unsupported value. Assuming 'ball'.");
            poiLocation = ballStore::getBall().getLocation();
        }
    }
    else
    {
        TRACE_ERROR("Parameter 'POI' not found. Assuming 'ball'.");
        poiLocation = ballStore::getBall().getLocation();
    }

    auto ownLocation = robotStore::getInstance().getOwnRobot().getLocation();
    auto obstacles = obstacleStore::getInstance().getAllObstaclesSortedByDistanceTo(ownLocation);

    if (obstacles.size() > 0)
    {
        auto closestObstacleLocation = obstacles.at(0).getLocation();
        auto polygon = polygon2D();

        if (std::abs(closestObstacleLocation.x - poiLocation.x) < std::abs(closestObstacleLocation.y - poiLocation.y))
        {
            polygon = polygon2D(
                    { Point2D(closestObstacleLocation.x - WIDTH, closestObstacleLocation.y),
                      Point2D(closestObstacleLocation.x + WIDTH, closestObstacleLocation.y),
                      Point2D(poiLocation.x + WIDTH, poiLocation.y),
                      Point2D(poiLocation.x - WIDTH, poiLocation.y) });
        }
        else
        {
            polygon = polygon2D(
                    { Point2D(closestObstacleLocation.x, closestObstacleLocation.y - WIDTH),
                      Point2D(closestObstacleLocation.x, closestObstacleLocation.y + WIDTH),
                      Point2D(poiLocation.x, poiLocation.y + WIDTH),
                      Point2D(poiLocation.x, poiLocation.y - WIDTH) });
        }

        for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
        {
            for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
            {
                if (polygon.pointExistsInPolygon(_heightMap(i, j)._center))
                {
                    _heightMap(i, j).setValue(heightMapValues::MAX);
                }
                else
                {
                    _heightMap(i, j).setValue(heightMapValues::NEUTRAL);
                }
            }
        }
    }

    return *this;
}

std::string hmBetweenPoiAndClosestObstacle::getDescription() const
{
    return "Between POI and closest obstacle";
}

std::string hmBetweenPoiAndClosestObstacle::getFilename() const
{
    return "hmBetweenPoiAndClosestObstacle";
}
