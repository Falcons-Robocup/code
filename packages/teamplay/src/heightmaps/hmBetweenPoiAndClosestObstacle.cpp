 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * hmBetweenPoiAndClosestObstacle.cpp
 *
 *  Created on: Jun 15, 2018
 *      Author: Coen Tempelaars
 */

#include "int/heightmaps/hmBetweenPoiAndClosestObstacle.hpp"

#include "FalconsCommon.h"
#include "polygon2D.hpp"

#include "int/stores/ballStore.hpp"
#include "int/stores/fieldDimensionsStore.hpp"
#include "int/stores/obstacleStore.hpp"
#include "int/stores/robotStore.hpp"
#include "int/utilities/trace.hpp"
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
