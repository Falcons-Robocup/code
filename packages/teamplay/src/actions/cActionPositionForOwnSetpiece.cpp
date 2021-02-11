// Copyright 2018-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionPositionForOwnSetpiece.cpp
 *
 *  Created on: Apr 24, 2018
 *      Author: Coen Tempelaars
 */

#include "int/actions/cActionPositionForOwnSetpiece.hpp"

#include "int/stores/heightMapStore.hpp"
#include "int/stores/robotStore.hpp"
#include "cDiagnostics.hpp"


using namespace teamplay;

cActionPositionForOwnSetpiece::cActionPositionForOwnSetpiece()
{
    boost::assign::insert( _actionParameters )
        ("onSide", std::make_pair(std::vector<std::string>{"withBall", "withoutBall"}, false));

    _intention.action = actionTypeEnum::MOVE;
}

cActionPositionForOwnSetpiece::~cActionPositionForOwnSetpiece()
{

}

/* \brief Position for own setpiece.
 *
 * \param onSide        The side that attracts the robot: withBall or withoutBall.
 *
 * \retval  RUNNING     if the robot is not at the target position
 *          PASSED      if the robot has reached the target position
 *          FAILED      if something went wrong (e.g. ball not found)
 */
behTreeReturnEnum cActionPositionForOwnSetpiece::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
        Position2D myPos = robotStore::getInstance().getOwnRobot().getPosition();

        // Get optimal location from heightmap
        auto optimum = heightMapStore::getInstance().getOptimum(CompositeHeightmapName::POSITION_FOR_OWN_SETPIECE, parameters);

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

        if (positionReached(targetPos.x, targetPos.y))
        {
            // Target reached. Do nothing and return PASSED
            TRACE("cActionPositionForOwnSetpiece PASSED");
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
        throw std::runtime_error(std::string("cActionPositionForOwnSetpiece::execute Linked to: ") + e.what());
    }

    return behTreeReturnEnum::FAILED;
}
