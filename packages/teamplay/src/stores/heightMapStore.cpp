 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * heightMapStore.cpp
 *
 *  Created on: Nov 9, 2017
 *      Author: Coen Tempelaars
 */

#include "int/stores/heightMapStore.hpp"

#include <stdexcept>

#include "int/heightmaps/hmAvoidBall.hpp"
#include "int/heightmaps/hmAvoidObstacles.hpp"
#include "int/heightmaps/hmAvoidTeamMates.hpp"
#include "int/heightmaps/hmBetweenPoiAndClosestObstacle.hpp"
#include "int/heightmaps/hmCloseToBallClaimedLocation.hpp"
#include "int/heightmaps/hmCloseToOwnPos.hpp"
#include "int/heightmaps/hmInFrontOfOppGoal.hpp"
#include "int/heightmaps/hmNearObstacles.hpp"
#include "int/heightmaps/hmNearOwnGoal.hpp"
#include "int/heightmaps/hmObstaclesBlockingBall.hpp"
#include "int/heightmaps/hmObstaclesBlockingOppGoal.hpp"
#include "int/heightmaps/hmObstaclesBlockingTeammates.hpp"

#include "int/stores/configurationStore.hpp"
#include "int/stores/robotStore.hpp"
#include "cDiagnostics.hpp"
#include "tracing.hpp"

#include "falconsCommon.hpp"

using namespace teamplay;


heightMapStore::heightMapStore()
{
    _heightmaps[heightmapEnum::AVOID_BALL] = std::make_shared<hmAvoidBall>();
    _heightmaps[heightmapEnum::AVOID_OBSTACLES] = std::make_shared<hmAvoidObstacles>();
    _heightmaps[heightmapEnum::AVOID_TEAM_MATES] = std::make_shared<hmAvoidTeamMates>();
    _heightmaps[heightmapEnum::BETWEEN_POI_AND_CLOSEST_OBSTACLE] = std::make_shared<hmBetweenPoiAndClosestObstacle>();
    _heightmaps[heightmapEnum::CLOSE_TO_BALL_CLAIMED_LOCATION] = std::make_shared<hmCloseToBallClaimedLocation>();
    _heightmaps[heightmapEnum::CLOSE_TO_OWN_POS] = std::make_shared<hmCloseToOwnPos>();
    _heightmaps[heightmapEnum::IN_FRONT_OF_OPP_GOAL] = std::make_shared<hmInFrontOfOppGoal>();
    _heightmaps[heightmapEnum::NEAR_OBSTACLES] = std::make_shared<hmNearObstacles>();
    _heightmaps[heightmapEnum::NEAR_OWN_GOAL] = std::make_shared<hmNearOwnGoal>();
    _heightmaps[heightmapEnum::OBS_BLOCKING_BALL] = std::make_shared<hmObstaclesBlockingBall>();
    _heightmaps[heightmapEnum::OBS_BLOCKING_OPP_GOAL] = std::make_shared<hmObstaclesBlockingOppGoal>();
    _heightmaps[heightmapEnum::OBS_BLOCKING_TEAMMATES] = std::make_shared<hmObstaclesBlockingTeammates>();
}

heightMapStore::~heightMapStore() { }

void heightMapStore::precalculateAll()
{
    TRACE_FUNCTION("");
    for (auto& kv : _heightmaps)
    {
        try
        {
            std::ostringstream msg;
            msg << kv.second->getFilename();
            TRACE_SCOPE("CALC_HEIGHTMAP", msg.str().c_str());

            kv.second->precalculate();
        }
        catch (std::exception& e)
        {
            TRACE_ERROR("Error calculating heightmap: %s: %s", kv.second->getFilename(), e.what());
            throw std::runtime_error(std::string("heightMapStore::calculateAll linked to: ") + e.what());
        }
    }
}

std::vector<std::string> heightMapStore::getDescriptions() const
{
    std::vector<std::string> descriptions;
    for (auto& kv : _heightmaps)
    {
        descriptions.push_back(kv.second->getDescription());
    }

    return descriptions;
}

abstractHeightMap heightMapStore::combineHeightmaps( const CompositeHeightmapName& name, const parameterMap_t& params ) const
{
    abstractHeightMap sum;

    for (auto& kv : _heightmaps)
    {
        try
        {
            float factor = configurationStore::getConfiguration().getHeightmapFactor(name, kv.first);

            if (factor > 0.0)
            {
                TRACE("  adding heightmap: ") << kv.second->getFilename() << " with factor: " << std::to_string(factor);
                sum = sum + kv.second->refine(params).scale(factor);
            }
        }
        catch (std::exception& e)
        {
            TRACE_ERROR("Error finding optimum while adding heightmap: %s: %s", kv.second->getFilename(), e.what());
            throw std::runtime_error(std::string("heightMapStore::getOptimum linked to: ") + e.what());
        }
    }

    return sum;
}

Point2D heightMapStore::getOptimum( const CompositeHeightmapName& name, const parameterMap_t& params ) const
{
    TRACE("Combine heightmaps and find optimum");

    auto sum = combineHeightmaps(name, params);

    TRACE("Found optimum at x: ") << std::to_string(sum.getOptimum()._center.x)
        << ", y: "  << std::to_string(sum.getOptimum()._center.y);

    return sum.getOptimum()._center;
}

Point2D heightMapStore::getOptimum( const CompositeHeightmapName& name, const parameterMap_t& params, const float tolerance ) const
{
    Point2D optimalLocation;
    
    TRACE("Combine heightmaps and find optimum with tolerance: %3.1f", tolerance);
    
    auto sum = combineHeightmaps(name, params);
    auto optimum = sum.getOptimum();
    auto own_location = robotStore::getInstance().getOwnRobot().getLocation();
    
    if (calc_distance(optimum._center, own_location) < heightMapValues::RESOLUTION)
    {
        TRACE("Optimal location is near own location");
        optimalLocation = own_location;
    }
    else
    {
        try
        {
            auto valueAtOptimalLocation = optimum.getValue();
            auto valueAtOwnLocation = sum.getFieldAtCoordinate(own_location).getValue();
            TRACE("Value at optimal location: %4.3f", valueAtOptimalLocation);
            TRACE("Value at own location: %4.3f", valueAtOwnLocation);
            if ((valueAtOptimalLocation - valueAtOwnLocation) <= tolerance)
            {
                optimalLocation = own_location;
            }
            else
            {
                optimalLocation = optimum._center;
            }
        }
        catch (std::exception&)
        {
            TRACE("Own location (%3.3f, %3.3f) is outside the heightmap", own_location.x, own_location.y);
            optimalLocation = optimum._center; // ignore the exception and return the calculated optimum
        }
    }
    
    return optimalLocation;
}

cv::Mat heightMapStore::generateOpenCVMatrix( const CompositeHeightmapName& name ) const
{
    parameterMap_t params;
    params["POI"] = "ball";  // Not so nice to hard code this parameter, but good enough for now
    auto sum = combineHeightmaps(name, params);
    auto image = sum.generateOpenCVMatrix();
    return image;
}
