 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
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
#include "int/utilities/trace.hpp"

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

abstractHeightMap heightMapStore::combineHeightmaps( const tpActionEnum& action, const parameterMap_t& params ) const
{
    abstractHeightMap sum;

    for (auto& kv : _heightmaps)
    {
        try
        {
            float factor = configurationStore::getConfiguration().getHeightMapFactorForAction(action, kv.first);

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

Point2D heightMapStore::getOptimum( const tpActionEnum& action, const parameterMap_t& params ) const
{
    TRACE("Combine heightmaps and find optimum");

    auto sum = combineHeightmaps(action, params);

    TRACE("Found optimum at x: ") << std::to_string(sum.getOptimum().x)
        << ", y: "  << std::to_string(sum.getOptimum().y);

    return sum.getOptimum();
}

void heightMapStore::generateJPG( const tpActionEnum& action, const std::string& filename, const parameterMap_t& params ) const
{
    auto sum = combineHeightmaps(action, params);

    if (teamplay::configurationStore::getConfiguration().getHeightMapsGeneratePictures())
    {
        sum.generateJPG(filename);
    }
}
