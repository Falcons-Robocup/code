// Copyright 2021 Erik Kouters (Falcons)
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

#include "int/stores/BallStore.hpp"
#include "int/stores/FieldDimensionsStore.hpp"
#include "int/stores/ObstacleStore.hpp"
#include "int/stores/RobotStore.hpp"
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
    _heightmapPrecalculated = true;
    return;
}

AbstractHeightMap hmBetweenPoiAndClosestObstacle::refine()
{
    auto myRole = teamplay::RobotStore::getInstance().getOwnRobot().getRole();
    auto poiLocation = Point2D();

    if (myRole == RoleEnum::ATTACKER_MAIN || myRole == RoleEnum::ATTACKER_ASSIST)
    {
        //attacker --> ball

        TRACE("hmBetweenPoiAndClosestObstacle: POI: ball");
        poiLocation = BallStore::getBall().getLocation();
    }
    else if (myRole == RoleEnum::DEFENDER_MAIN || myRole == RoleEnum::DEFENDER_ASSIST)
    {
        //defender --> own goal

        TRACE("hmBetweenPoiAndClosestObstacle: POI: P_OWN_GOALLINE_CENTER");
        poiLocation = FieldDimensionsStore::getFieldDimensions().getLocation(FieldPOI::OWN_GOALLINE_CENTER);
    }
    else
    {
        TRACE_ERROR("hmBetweenPoiAndClosestObstacle with unexpected role. Assuming 'ball'.");
        poiLocation = BallStore::getBall().getLocation();
    }

    auto ownLocation = RobotStore::getInstance().getOwnRobot().getLocation();
    auto obstacles = ObstacleStore::getInstance().getAllObstaclesSortedByDistanceTo(ownLocation);

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
                    _heightMap(i, j).setValue(HeightMapValues::MAX);
                }
                else
                {
                    _heightMap(i, j).setValue(HeightMapValues::NEUTRAL);
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
