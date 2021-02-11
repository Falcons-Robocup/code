// Copyright 2018-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionDefendAttackingOpponent.cpp
 *
 *  Created on: Jun 13, 2018
 *      Author: Coen Tempelaars
 */

#include "int/actions/cActionDefendAttackingOpponent.hpp"

#include "int/stores/heightMapStore.hpp"
#include "int/stores/robotStore.hpp"
#include "cDiagnostics.hpp"


using namespace teamplay;

cActionDefendAttackingOpponent::cActionDefendAttackingOpponent()
{
    boost::assign::insert( _actionParameters )
    ("POI", std::make_pair(std::vector<std::string>{"ball", "P_OWN_GOALLINE_CENTER"}, false));

    _intention.action = actionTypeEnum::MOVE;
}

cActionDefendAttackingOpponent::~cActionDefendAttackingOpponent()
{

}

/* \brief Defend attacking opponent by projecting a line between the attacking opponent
 *        and the POI. The target is placed on this line, near the attacking opponent.
 *
 * \param   POI         "ball" or "P_OWN_GOALLINE_CENTER"
 *
 * \retval  RUNNING     if the robot is not at the target position
 *          PASSED      if the robot has reached the target position
 *          FAILED      if something went wrong (e.g. ball not found)
 */
behTreeReturnEnum cActionDefendAttackingOpponent::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
        Position2D myPos = robotStore::getInstance().getOwnRobot().getPosition();

        // Get optimal location from heightmap
        auto optimum = heightMapStore::getInstance().getOptimum(CompositeHeightmapName::DEFEND_ATTACKING_OPPONENT, parameters);

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
            TRACE("cActionDefendAttackingOpponent PASSED");
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
        throw std::runtime_error(std::string("cActionDefendAttackingOpponent::execute Linked to: ") + e.what());
    }

    return behTreeReturnEnum::FAILED;
}
