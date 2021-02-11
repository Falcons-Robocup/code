// Copyright 2016-2020 martijn van veen (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionAvoidPOI.cpp
 *
 *  Created on: Jun 27, 2016
 *      Author: Martijn van Veen
 */

#include "int/actions/cActionAvoidPOI.hpp"
#include "int/stores/ballStore.hpp"
#include "cDiagnostics.hpp"


cActionAvoidPOI::cActionAvoidPOI()
{
    boost::assign::insert( _actionParameters )
    ("target", std::make_pair(defaultPOI, false) )
        ;
    _intention.action = actionTypeEnum::UNKNOWN;
}

cActionAvoidPOI::~cActionAvoidPOI()
{

}

// Add POI as obstacle
// Returns: PASSED    if the obstacle is set in the ballstore (which will be sent to pathplanning if a move action occurs)
//          FAILED    if something went wrong // can this happen/does worldmodel tell us?
//			RUNNING   is never returned
behTreeReturnEnum cActionAvoidPOI::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
        std::string targetStr("target");
    	boost::optional<Position2D> target = getPos2DFromStr(parameters, targetStr);

        if (target)
        {
            Position2D targetPos = *target;

            _intention.position.x = targetPos.x;
            _intention.position.y = targetPos.y;
            sendIntention();

            teamplay::ballStore::getBall().avoid();
            TRACE("cActionAvoidPOI PASSED");
            return behTreeReturnEnum::PASSED;
        }
        else
        {
            // No valid POI given to avoid
        	TRACE("cActionAvoidPOI Failed: no valid POI given to avoid");
            return behTreeReturnEnum::PASSED;
        }
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("cActionAvoidPOI::execute Linked to: ") + e.what());
    }

    return behTreeReturnEnum::FAILED;
}
