// Copyright 2018-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionTurnAwayFromOpponent.cpp
 *
 *  Created on: Apr 28, 2018
 *      Author: Erik Kouters
 */

#include "int/actions/cActionTurnAwayFromOpponent.hpp"

#include <string>

#include "falconsCommon.hpp"
#include "int/stores/ballStore.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/stores/robotStore.hpp"
#include "cDiagnostics.hpp"


using namespace teamplay;

cActionTurnAwayFromOpponent::cActionTurnAwayFromOpponent()
{
    boost::assign::insert( _actionParameters )
        ("target", std::make_pair(defaultPOI, false) )
        ("motionProfile", std::make_pair(std::vector<std::string>{defaultMotionProfiles}, true))
        ;

    _intention.action = actionTypeEnum::TURN_AWAY_FROM_OPPONENT;
}

cActionTurnAwayFromOpponent::~cActionTurnAwayFromOpponent()
{

}

/* \brief Turn away from 'target' (x,y)
 *
 * \param target              The target (x,y)
 *
 * \retval  RUNNING     if the robot is not at the target position
 *          PASSED      if the robot has reached the target position
 *          FAILED      if something went wrong
 */
behTreeReturnEnum cActionTurnAwayFromOpponent::execute(const std::map<std::string, std::string> &parameters)
{
    behTreeReturnEnum result = behTreeReturnEnum::RUNNING;
    try
    {
        // parameter "target" is the place to go.
        std::string targetStr("target");
        boost::optional<Position2D> target = getPos2DFromStr(parameters, targetStr);

        if (target)
        {
            Position2D targetPos = *target;

            // Send in the intention the (x,y) of the opponent we are looking away from
            _intention.position.x = targetPos.x;
            _intention.position.y = targetPos.y;
            sendIntention();

            // ensure that we move according to the rules
            if (!isCurrentPosValid())
            {
                // Move towards the center, while maintaining angle and motion profile
                moveTo(0.0, 0.0);
                return behTreeReturnEnum::RUNNING;
            }

            if (!isTargetPosInsideSafetyBoundaries(targetPos))
            {
                return behTreeReturnEnum::FAILED;
            }

            result = turnAwayFromOpponent(targetPos.x, targetPos.y);
            return result;
        }

    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("cActionTurnAwayFromOpponent::execute Linked to: ") + e.what());
    }

    return behTreeReturnEnum::FAILED;
}
