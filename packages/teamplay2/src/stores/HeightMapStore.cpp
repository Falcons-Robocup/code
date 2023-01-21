// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * HeightMapStore.cpp
 *
 *  Created on: Nov 9, 2017
 *      Author: Coen Tempelaars
 */

#include "int/stores/HeightMapStore.hpp"

#include <stdexcept>
#include <thread>

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

#include "int/stores/ConfigurationStore.hpp"
#include "int/stores/RobotStore.hpp"

#include "cDiagnostics.hpp"
#include "tracing.hpp"

#include "falconsCommon.hpp"

using namespace teamplay;


HeightMapStore::HeightMapStore()
{
    _heightmaps[HeightmapEnum::AVOID_BALL] = std::make_shared<hmAvoidBall>();
    _heightmaps[HeightmapEnum::AVOID_OBSTACLES] = std::make_shared<hmAvoidObstacles>();
    _heightmaps[HeightmapEnum::AVOID_TEAM_MATES] = std::make_shared<hmAvoidTeamMates>();
    _heightmaps[HeightmapEnum::BETWEEN_POI_AND_CLOSEST_OBSTACLE] = std::make_shared<hmBetweenPoiAndClosestObstacle>();
    _heightmaps[HeightmapEnum::CLOSE_TO_BALL_CLAIMED_LOCATION] = std::make_shared<hmCloseToBallClaimedLocation>();
    _heightmaps[HeightmapEnum::CLOSE_TO_OWN_POS] = std::make_shared<hmCloseToOwnPos>();
    _heightmaps[HeightmapEnum::IN_FRONT_OF_OPP_GOAL] = std::make_shared<hmInFrontOfOppGoal>();
    _heightmaps[HeightmapEnum::NEAR_OBSTACLES] = std::make_shared<hmNearObstacles>();
    _heightmaps[HeightmapEnum::NEAR_OWN_GOAL] = std::make_shared<hmNearOwnGoal>();
    _heightmaps[HeightmapEnum::OBS_BLOCKING_BALL] = std::make_shared<hmObstaclesBlockingBall>();
    _heightmaps[HeightmapEnum::OBS_BLOCKING_OPP_GOAL] = std::make_shared<hmObstaclesBlockingOppGoal>();
    _heightmaps[HeightmapEnum::OBS_BLOCKING_TEAMMATES] = std::make_shared<hmObstaclesBlockingTeammates>();
}

HeightMapStore::~HeightMapStore() { }

void HeightMapStore::resetHeightmapPrecalculations()
{
    TRACE_FUNCTION("");

    for (auto& kv : _heightmaps)
    {
        kv.second->resetHeightmapPrecalculation();
    }
}

std::vector<std::string> HeightMapStore::getDescriptions() const
{
    std::vector<std::string> descriptions;
    for (auto& kv : _heightmaps)
    {
        descriptions.push_back(kv.second->getDescription());
    }

    return descriptions;
}

AbstractHeightMap HeightMapStore::combineHeightmaps( const CompositeHeightmapName& name ) const
{
    TRACE_FUNCTION(enum2str(name));
    AbstractHeightMap sum;

    for (auto& kv : _heightmaps)
    {
        TRACE_SCOPE("COMBINE_HEIGHTMAP", enum2str(kv.first));
        try
        {
            float factor = ConfigurationStore::getConfiguration().getHeightmapFactor(name, kv.first);

            if (factor > 0.0)
            {
                // first precalculate heightmap before use
                kv.second->precalculate();

                TRACE("  adding heightmap: ") << kv.second->getFilename() << " with factor: " << std::to_string(factor);
                sum = sum + kv.second->refine().scale(factor);
            }
        }
        catch (std::exception& e)
        {
            TRACE_ERROR("Error finding optimum while adding heightmap: _heightmaps[%d] = %s: %s", kv.first, kv.second->getFilename(), e.what());
            throw std::runtime_error(std::string("HeightMapStore::getOptimum linked to: ") + e.what());
        }
    }

    return sum;
}

Point2D HeightMapStore::getOptimum( const CompositeHeightmapName& name ) const
{
    TRACE_FUNCTION("Combine heightmaps and find optimum");

    auto sum = combineHeightmaps(name);

    TRACE("Found optimum at x: ") << std::to_string(sum.getOptimum()._center.x)
        << ", y: "  << std::to_string(sum.getOptimum()._center.y);

    return sum.getOptimum()._center;
}

cv::Mat HeightMapStore::generateOpenCVMatrix( const CompositeHeightmapName& name ) const
{
    auto sum = combineHeightmaps(name);
    auto image = sum.generateOpenCVMatrix();
    return image;
}