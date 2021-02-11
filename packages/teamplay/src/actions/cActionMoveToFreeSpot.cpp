// Copyright 2016-2020 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionMoveToFreeSpot.cpp
 *
 *  Created on: May 22, 2016
 *      Author: Tim Kouters
 */

#include "int/actions/cActionMoveToFreeSpot.hpp"

#include <string>
#include <vector>

#include "falconsCommon.hpp"
#include "int/stores/ballStore.hpp"
#include "int/stores/heightMapStore.hpp"
#include "int/stores/robotStore.hpp"
#include "cDiagnostics.hpp"


using namespace teamplay;

cActionMoveToFreeSpot::cActionMoveToFreeSpot()
{
    boost::assign::insert( _actionParameters )
    ("distanceThreshold", std::make_pair(std::vector<std::string>{"float"}, true) )
    ("areaOfInterest", std::make_pair(defaultArea, true))
    ;

    _intention.action = actionTypeEnum::MOVE;
}

cActionMoveToFreeSpot::~cActionMoveToFreeSpot()
{

}

/* \brief Move to a free spot in a certain area of interest
 *
 * \param areaOfInterest      The applicable area
 * \param distanceThreshold   [Optional] The allowed delta between the target position and current position
 *
 * \retval  RUNNING     if the robot is not at the target position
 *          PASSED      if the robot has reached the target position
 *          FAILED      if something went wrong
 */
behTreeReturnEnum cActionMoveToFreeSpot::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
        Position2D myPos = robotStore::getInstance().getOwnRobot().getPosition();
        ball ball = ballStore::getBall(); // retrieve ball position information
        Vector2D ballPos(ball.getPosition().x, ball.getPosition().y);

        // Get optimal location from heightmap
        Point2D optimum = heightMapStore::getInstance().getOptimum(CompositeHeightmapName::MOVE_TO_FREE_SPOT, parameters);

        // Compute target positions
        Position2D targetPos;
        targetPos.x = optimum.x;
        targetPos.y = optimum.y;

        // parameter "distanceThreshold" is the allowed delta between the target and the real position
        std::string distanceThresholdStr("distanceThreshold");
        double distanceThreshold = XYpositionTolerance;
        auto paramValPair = parameters.find(distanceThresholdStr);
        if (paramValPair != parameters.end())
        {
            std::string distanceThresholdVal = paramValPair->second;
            if (distanceThresholdVal.compare(emptyValue) != 0)
            {
                distanceThreshold = std::stod(distanceThresholdVal);
            }
        }

        _intention.position.x = targetPos.x;
        _intention.position.y = targetPos.y;
        sendIntention();

        // ensure that we move according to the rules
        if (!isCurrentPosValid())
        {
            // Move towards the center, while maintaining angle
            moveTo(0.0, 0.0);
            return behTreeReturnEnum::RUNNING;
        }

        if (!isTargetPosInsideSafetyBoundaries(targetPos))
        {
            return behTreeReturnEnum::FAILED;
        }

        if (positionReached(targetPos.x, targetPos.y, distanceThreshold))
        {
            // Target reached. Do nothing and return PASSED
            TRACE("cActionMoveToFreeSpot PASSED");
            moveTo(myPos.x, myPos.y);
            return behTreeReturnEnum::PASSED;
        }
        else
        {
            // Target not reached. moveTo and return RUNNING
            moveTo(targetPos.x, targetPos.y);
            return behTreeReturnEnum::RUNNING;
        }

    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("cActionMoveToFreeSpot::execute Linked to: ") + e.what());
    }

    return behTreeReturnEnum::FAILED;
}
