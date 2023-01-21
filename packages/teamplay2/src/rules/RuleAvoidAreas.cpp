// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RuleAvoidAreas.cpp
 *
 *  Created on: Apr 24, 2017
 *      Author: Coen Tempelaars
 */

#include "int/rules/RuleAvoidAreas.hpp"

#include "int/stores/FieldDimensionsStore.hpp"
#include "int/stores/RobotStore.hpp"
#include "tracing.hpp"

using namespace teamplay;

//After 10.0 seconds, the Robot must have the *intention* to drive away from the area
static const double timeout = 10.0;


RuleAvoidAreas::RuleAvoidAreas(boost::shared_ptr<teamplay::Timer> t)
{
    _timer = t;
}

RuleAvoidAreas::~RuleAvoidAreas()
{
    _timer.reset();  //This releases the shared pointer
}

bool RuleAvoidAreas::isCurrentPositionValid() const
{
    auto current_position = RobotStore::getInstance().getOwnRobot().getPosition();
    if (FieldDimensionsStore::getFieldDimensions().isPositionInArea(current_position, FieldArea::OWN_GOALAREA))
    {
        return isCurrentPositionInOwnGoalAreaValid();
    }

    if (FieldDimensionsStore::getFieldDimensions().isPositionInArea(current_position, FieldArea::OPP_GOALAREA))
    {
        return isCurrentPositionInOppGoalAreaValid();
    }

    if (FieldDimensionsStore::getFieldDimensions().isPositionInArea(current_position, FieldArea::OWN_PENALTYAREA))
    {
        return isCurrentPositionInOwnPenaltyAreaValid();
    }

    if (FieldDimensionsStore::getFieldDimensions().isPositionInArea(current_position, FieldArea::OPP_PENALTYAREA))
    {
        return isCurrentPositionInOppPenaltyAreaValid();
    }

    TRACE("Current position is valid, because it is not in an area of interest");
    _timer->reset();
    return true;
}

bool RuleAvoidAreas::isCurrentPositionInOwnPenaltyAreaValid() const
{
    /* 1. The goalie is always allowed to be in the own penalty area
     * 2. One field player is allowed to be in the own penalty area
     *    as long as the Timer has not elapsed */

    if (RobotStore::getInstance().getOwnRobot().getRole() != RoleEnum::GOALKEEPER)
    {
        if (RobotStore::getInstance().getAllRobotsExclGoalieInArea(FieldArea::OWN_PENALTYAREA).size() > 1)
        {
            TRACE("Current position is invalid, because too many teammates in own penalty area");
            return false;
        }

        if (_timer->hasElapsed(timeout))
        {
            TRACE("Current position is invalid, because the Timer has elapsed");
            return false;
        }
    }

    TRACE("Current position is valid");
    return true;
}

bool RuleAvoidAreas::isCurrentPositionInOppPenaltyAreaValid() const
{
    /* One field player is allowed to be in the opponent penalty area
     * as long as the Timer has not elapsed */

    if (RobotStore::getInstance().getAllRobotsInArea(FieldArea::OPP_PENALTYAREA).size() > 1)
    {
        TRACE("Current position is invalid, because too many teammates in opponent penalty area");
        return false;
    }

    if (_timer->hasElapsed(timeout))
    {
        TRACE("Current position is invalid, because the Timer has elapsed");
        return false;
    }

    TRACE("Current position is valid");
    return true;
}

bool RuleAvoidAreas::isCurrentPositionInOwnGoalAreaValid() const
{
    /* Only the goalie is allowed in the own goal area */

    if (RobotStore::getInstance().getOwnRobot().getRole() != RoleEnum::GOALKEEPER)
    {
        TRACE("Current position is invalid, because this Robot is not goalkeeper");
        return false;
    }

    TRACE("Current position is valid");
    return true;
}

bool RuleAvoidAreas::isCurrentPositionInOppGoalAreaValid() const
{
    /* No-one is allowed in the opponent goal area */

    TRACE("Current position is invalid, no-one is allowed in the opponent goal area");
    return false;
}

std::vector<polygon2D> RuleAvoidAreas::getForbiddenAreas() const
{
    std::vector<polygon2D> forbidden_areas;
    auto currentPosition = RobotStore::getInstance().getOwnRobot().getPosition();

    if (!isCurrentPositionInOwnGoalAreaValid())
    {
        TRACE("FBA: OWN_GOALAREA");
        forbidden_areas.push_back(FieldDimensionsStore::getFieldDimensions().getArea(FieldArea::OWN_GOALAREA));
    }

    if (!isCurrentPositionInOppGoalAreaValid())
    {
        TRACE("FBA: OPP_GOALAREA");
        forbidden_areas.push_back(FieldDimensionsStore::getFieldDimensions().getArea(FieldArea::OPP_GOALAREA));
    }

    if (FieldDimensionsStore::getFieldDimensions().isPositionNearArea(currentPosition, FieldArea::OWN_PENALTYAREA))
    {
        if (!isCurrentPositionInOwnPenaltyAreaValid())
        {
            TRACE("FBA: OWN_PENALTYAREA");
            forbidden_areas.push_back(FieldDimensionsStore::getFieldDimensions().getArea(FieldArea::OWN_PENALTYAREA));
        }
    }

    if (FieldDimensionsStore::getFieldDimensions().isPositionNearArea(currentPosition, FieldArea::OPP_PENALTYAREA))
    {
        if (!isCurrentPositionInOppPenaltyAreaValid())
        {
            TRACE("FBA: OPP_PENALTYAREA");
            forbidden_areas.push_back(FieldDimensionsStore::getFieldDimensions().getArea(FieldArea::OPP_PENALTYAREA));
        }
    }

    return forbidden_areas;
}
