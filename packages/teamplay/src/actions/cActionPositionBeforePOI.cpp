// Copyright 2016-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionPositionBeforePOI.cpp
 *
 *  Created on: May 3, 2016
 *      Author: Erik Kouters
 */

#include "int/actions/cActionPositionBeforePOI.hpp"

#include "falconsCommon.hpp"
#include "int/stores/robotStore.hpp"
#include "cDiagnostics.hpp"


cActionPositionBeforePOI::cActionPositionBeforePOI()
{
    boost::assign::insert( _actionParameters )
        ("source", std::make_pair(defaultPOI, false))
        ("destination", std::make_pair(defaultPOI, false))
        ("distance", std::make_pair(std::vector<std::string>{"float"}, false))
        ("distanceThreshold", std::make_pair(std::vector<std::string>{"float"}, true) )
        ("motionProfile", std::make_pair(std::vector<std::string>{defaultMotionProfiles}, true))
        ;

    _intention.action = actionTypeEnum::MOVE;
}

cActionPositionBeforePOI::~cActionPositionBeforePOI()
{

}

/* \brief Project a line from a source POI to a destination POI and place target location before destination POI.
 * The target location is placed 'distance' meters before the destination POI, taking 'distanceThreshold' into account.
 * Optionally, turn to face the optional argument 'facing' POI.
 *
 * \param source              The source POI for the line projection.
 * \param destination         The destination POI for the line projection.
 * \param distance            The distance between the target location and the POI.
 * \param distanceThreshold   [Optional] The allowed delta between the target position and current position
  *
 * \retval  RUNNING     if the robot is not at the target position
 *          PASSED      if the robot has reached the target position
 *          FAILED      if something went wrong (e.g. ball not found)
 */
behTreeReturnEnum cActionPositionBeforePOI::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
        // Own robot position
    	Position2D myPos = teamplay::robotStore::getInstance().getOwnRobot().getPosition();

    	// The source POI for the line projection.
        std::string sourceStr("source");
        boost::optional<Position2D> source = getPos2DFromStr(parameters, sourceStr);

        // The destination POI for the line projection.
        std::string destinationStr("destination");
        boost::optional<Position2D> destination = getPos2DFromStr(parameters, destinationStr);

        // The distance between the target location and the POI.
        std::string paramStr("distance");
        double dist = -1.0;
        auto paramValPair = parameters.find(paramStr);
        if (paramValPair != parameters.end())
        {
            std::string paramVal = paramValPair->second;
            dist = std::stod(paramVal);
        }

        // parameter "distanceThreshold" is the allowed delta between the target and the real position
        std::string distanceThresholdStr("distanceThreshold");
        double distanceThreshold = XYpositionTolerance;
        paramValPair = parameters.find(distanceThresholdStr);
        if (paramValPair != parameters.end())
        {
            std::string distanceThresholdVal = paramValPair->second;
            if (distanceThresholdVal.compare(emptyValue) != 0)
            {
                distanceThreshold = std::stod(distanceThresholdVal);
            }
        }

        // parameter "motionProfile" defines with which profile to move with (e.g., normal play or more careful during a setpiece)
        std::string motionProfileStr("motionProfile");
        std::string motionProfileValue = "normal";
        paramValPair = parameters.find(motionProfileStr);
        if (paramValPair != parameters.end())
        {
            motionProfileValue = paramValPair->second;
        }

        // Must have 'source', 'destination', and 'distance'
        if (source && destination && dist > 0.0)
        {
            Vector2D sourceVec = source->xy();
            Vector2D destVec = destination->xy();

            Vector2D targetVec = (1 - (dist / (destVec - sourceVec).size())) * (destVec - sourceVec) + sourceVec;
            Position2D targetPos = Position2D(targetVec.x, targetVec.y, 0.0);

            _intention.position.x = targetPos.x;
            _intention.position.y = targetPos.y;
            sendIntention();

            if (positionReached(targetPos.x, targetPos.y, distanceThreshold))
            {
                // Target reached. Do nothing and return PASSED
                TRACE("cActionPositionBeforeBall PASSED");
                moveTo(myPos.x, myPos.y);
                return behTreeReturnEnum::PASSED;
            }
            else
            {
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

                // Target not reached. moveTo and return RUNNING
                moveTo(targetPos.x, targetPos.y, motionProfileValue);
                return behTreeReturnEnum::RUNNING;
            }

        }

    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("cActionPositionBeforePOI::execute Linked to: ") + e.what());
    }

    return behTreeReturnEnum::FAILED;
}
