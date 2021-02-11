// Copyright 2017-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ruleAvoidAreas.cpp
 *
 *  Created on: Apr 24, 2017
 *      Author: Coen Tempelaars
 */

#include "int/rules/ruleAvoidAreas.hpp"

#include "int/stores/fieldDimensionsStore.hpp"
#include "int/stores/robotStore.hpp"
#include "tracing.hpp"

using namespace teamplay;

//After 10.0 seconds, the robot must have the *intention* to drive away from the area
static const double timeout = 10.0;


ruleAvoidAreas::ruleAvoidAreas(boost::shared_ptr<teamplay::timer> t)
{
    _timer = t;
}

ruleAvoidAreas::~ruleAvoidAreas()
{
    _timer.reset();  //This releases the shared pointer
}

bool ruleAvoidAreas::isCurrentPositionValid() const
{
    auto current_position = robotStore::getInstance().getOwnRobot().getPosition();
    if (fieldDimensionsStore::getFieldDimensions().isPositionInArea(current_position, fieldArea::OWN_GOALAREA))
    {
        return isCurrentPositionInOwnGoalAreaValid();
    }

    if (fieldDimensionsStore::getFieldDimensions().isPositionInArea(current_position, fieldArea::OPP_GOALAREA))
    {
        return isCurrentPositionInOppGoalAreaValid();
    }

    if (fieldDimensionsStore::getFieldDimensions().isPositionInArea(current_position, fieldArea::OWN_PENALTYAREA))
    {
        return isCurrentPositionInOwnPenaltyAreaValid();
    }

    if (fieldDimensionsStore::getFieldDimensions().isPositionInArea(current_position, fieldArea::OPP_PENALTYAREA))
    {
        return isCurrentPositionInOppPenaltyAreaValid();
    }

    TRACE("Current position is valid, because it is not in an area of interest");
    _timer->reset();
    return true;
}

bool ruleAvoidAreas::isCurrentPositionInOwnPenaltyAreaValid() const
{
    /* 1. The goalie is always allowed to be in the own penalty area
     * 2. One field player is allowed to be in the own penalty area
     *    as long as the timer has not elapsed */

    if (robotStore::getInstance().getOwnRobot().getRole() != treeEnum::R_GOALKEEPER)
    {
        if (robotStore::getInstance().getAllRobotsExclGoalieInArea(fieldArea::OWN_PENALTYAREA).size() > 1)
        {
            TRACE("Current position is invalid, because too many teammates in own penalty area");
            return false;
        }

        if (_timer->hasElapsed(timeout))
        {
            TRACE("Current position is invalid, because the timer has elapsed");
            return false;
        }
    }

    TRACE("Current position is valid");
    return true;
}

bool ruleAvoidAreas::isCurrentPositionInOppPenaltyAreaValid() const
{
    /* One field player is allowed to be in the opponent penalty area
     * as long as the timer has not elapsed */

    if (robotStore::getInstance().getAllRobotsInArea(fieldArea::OPP_PENALTYAREA).size() > 1)
    {
        TRACE("Current position is invalid, because too many teammates in opponent penalty area");
        return false;
    }

    if (_timer->hasElapsed(timeout))
    {
        TRACE("Current position is invalid, because the timer has elapsed");
        return false;
    }

    TRACE("Current position is valid");
    return true;
}

bool ruleAvoidAreas::isCurrentPositionInOwnGoalAreaValid() const
{
    /* Only the goalie is allowed in the own goal area */

    if (robotStore::getInstance().getOwnRobot().getRole() != treeEnum::R_GOALKEEPER)
    {
        TRACE("Current position is invalid, because this robot is not goalkeeper");
        return false;
    }

    TRACE("Current position is valid");
    return true;
}

bool ruleAvoidAreas::isCurrentPositionInOppGoalAreaValid() const
{
    /* No-one is allowed in the opponent goal area */

    TRACE("Current position is invalid, no-one is allowed in the opponent goal area");
    return false;
}

std::vector<polygon2D> ruleAvoidAreas::getForbiddenAreas() const
{
    std::vector<polygon2D> forbidden_areas;
    auto currentPosition = robotStore::getInstance().getOwnRobot().getPosition();

    if (!isCurrentPositionInOwnGoalAreaValid())
    {
        TRACE("FBA: OWN_GOALAREA");
        forbidden_areas.push_back(fieldDimensionsStore::getFieldDimensions().getArea(fieldArea::OWN_GOALAREA));
    }

    if (!isCurrentPositionInOppGoalAreaValid())
    {
        TRACE("FBA: OPP_GOALAREA");
        forbidden_areas.push_back(fieldDimensionsStore::getFieldDimensions().getArea(fieldArea::OPP_GOALAREA));
    }

    if (fieldDimensionsStore::getFieldDimensions().isPositionNearArea(currentPosition, fieldArea::OWN_PENALTYAREA))
    {
        if (!isCurrentPositionInOwnPenaltyAreaValid())
        {
            TRACE("FBA: OWN_PENALTYAREA");
            forbidden_areas.push_back(fieldDimensionsStore::getFieldDimensions().getArea(fieldArea::OWN_PENALTYAREA));
        }
    }

    if (fieldDimensionsStore::getFieldDimensions().isPositionNearArea(currentPosition, fieldArea::OPP_PENALTYAREA))
    {
        if (!isCurrentPositionInOppPenaltyAreaValid())
        {
            TRACE("FBA: OPP_PENALTYAREA");
            forbidden_areas.push_back(fieldDimensionsStore::getFieldDimensions().getArea(fieldArea::OPP_PENALTYAREA));
        }
    }

    return forbidden_areas;
}
