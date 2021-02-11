// Copyright 2016-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionGetBall.cpp
 *
 *  Created on: May 4, 2016
 *      Author: Erik Kouters
 */

#include "falconsCommon.hpp"
#include "int/actions/cActionGetBall.hpp"
#include "int/stores/ballStore.hpp"
#include "cDiagnostics.hpp"
#include "int/stores/robotStore.hpp"

using namespace teamplay;

cActionGetBall::cActionGetBall()
{
    boost::assign::insert( _actionParameters )
        ("motionProfile", std::make_pair(std::vector<std::string>{defaultMotionProfiles}, true))
        ;

    _intention.action = actionTypeEnum::GET_BALL;
}

cActionGetBall::~cActionGetBall()
{

}

// Move to ballPos (x,y).
// Turn to look towards 'facing' (x,y).
// If no 'facing' was given, turn to look towards the ball.
// If no ball was found, do not turn (keep current facing).
// Returns: RUNNING   if the robot is not at the target position
//          PASSED    if the robot reached the target position
//          FAILED    if something went wrong
behTreeReturnEnum cActionGetBall::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
        // ensure that we move according to the rules
        if (!isCurrentPosValid())
        {
            // Move towards the center, while maintaining angle
            moveTo(0.0, 0.0);
            return behTreeReturnEnum::RUNNING;
        }

        // parameter "motionProfile" defines with which profile to move with (e.g., normal play or more careful during a setpiece)
        std::string motionProfileStr("motionProfile");
        std::string motionProfileValue = "normal";
        auto paramValPair = parameters.find(motionProfileStr);
        if (paramValPair != parameters.end())
        {
            motionProfileValue = paramValPair->second;
        }

        sendIntention();

        return getBall(motionProfileValue);
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("cActionGetBall::execute Linked to: ") + e.what());
    }

    return behTreeReturnEnum::FAILED;
}

