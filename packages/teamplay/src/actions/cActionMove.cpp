// Copyright 2016-2020 Michel Koenen (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionMove.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: Ivo Matthijssen
 */

#include "int/actions/cActionMove.hpp"

#include <string>

#include "falconsCommon.hpp"
#include "int/stores/ballStore.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/stores/robotStore.hpp"
#include "cDiagnostics.hpp"


using namespace teamplay;

cActionMove::cActionMove()
{
    boost::assign::insert( _actionParameters )
        ("target", std::make_pair(defaultPOI, false) )
        ("distanceThreshold", std::make_pair(std::vector<std::string>{"float"}, true) )
        ("motionProfile", std::make_pair(std::vector<std::string>{defaultMotionProfiles}, true))
        ;

    _intention.action = actionTypeEnum::MOVE;
}

cActionMove::~cActionMove()
{

}

/* \brief Move to 'target' (x,y)
 *
 * \param target              The target (x,y)
 * \param distanceThreshold   [Optional] The allowed delta between the target position and current position
 *
 * \retval  RUNNING     if the robot is not at the target position
 *          PASSED      if the robot has reached the target position
 *          FAILED      if something went wrong
 */
behTreeReturnEnum cActionMove::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
        // parameter "target" is the place to go.
        std::string targetStr("target");
        boost::optional<Position2D> target = getPos2DFromStr(parameters, targetStr);

        // [Optional] parameter "motionProfile" defines with which profile to move with (e.g., normal play or more careful during a setpiece)
        std::string motionProfileStr("motionProfile");
        std::string motionProfileValue = "normal";
        auto paramValPair = parameters.find(motionProfileStr);
        if (paramValPair != parameters.end())
        {
            motionProfileValue = paramValPair->second;
        }

        if (target)
        {
            Position2D targetPos = *target;

            _intention.position.x = targetPos.x;
            _intention.position.y = targetPos.y;
            sendIntention();

            // ensure that we move according to the rules
            if (!isCurrentPosValid())
            {
                // Move towards the center, while maintaining angle and motion profile
                moveTo(0.0, 0.0, motionProfileValue);
                return behTreeReturnEnum::RUNNING;
            }

            if (!isTargetPosInsideSafetyBoundaries(targetPos))
            {
                return behTreeReturnEnum::FAILED;
            }

            // Previously determined targetPos.
            // Now move there
            return moveTo(targetPos.x, targetPos.y, motionProfileValue);
        }

    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("cActionMove::execute Linked to: ") + e.what());
    }

    return behTreeReturnEnum::FAILED;
}
