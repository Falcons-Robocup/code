// Copyright 2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionDribble.cpp
 *
 *  Created on: Feb 15, 2020
 *      Author: Coen Tempelaars
 */

#include "int/actions/cActionDribble.hpp"

#include "int/stores/heightMapStore.hpp"
#include "int/stores/robotStore.hpp"
#include "cDiagnostics.hpp"


using namespace teamplay;


cActionDribble::cActionDribble()
{
    _intention.action = actionTypeEnum::MOVE;
}

cActionDribble::~cActionDribble()
{

}


/* \brief Move to a position that is suitable for shooting, lobbing or passing.
          If the current position is close enough to optimal, stay in place.
 *
 * \retval  RUNNING     if the current position is not close enough to optimal
 *          PASSED      if the current position is close enough to optimal
 *          FAILED      if something went wrong (e.g. ball not found)
 */
behTreeReturnEnum cActionDribble::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
        // Get optimal location from heightmap, using a tolerance to avoid driving long distances to reach only a slighter higher value
        float tolerance = 20.0;  // TODO: make configurable?
        auto optimum = heightMapStore::getInstance().getOptimum(CompositeHeightmapName::DRIBBLE, parameters, tolerance);

        if (positionReached(optimum.x, optimum.y))
        {
            TRACE("INFO: cActionDribble: target position reached");
            return behTreeReturnEnum::PASSED;
        }
        else /* target position not reached */
        {
            TRACE("INFO: cActionDribble: target position not reached");

            // Compute target positions
            Position2D targetPos;
            targetPos.x = optimum.x;
            targetPos.y = optimum.y;

            // Send intention
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

            // Target not reached. moveTo and return RUNNING
            moveTo(targetPos.x, targetPos.y);
            return behTreeReturnEnum::RUNNING;
        }
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("cActionDribble::execute Linked to: ") + e.what());
    }

    return behTreeReturnEnum::FAILED;
}
