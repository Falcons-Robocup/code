 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ruleAvoidAreas.cpp
 *
 *  Created on: Apr 24, 2017
 *      Author: Coen Tempelaars
 */

#include "int/rules/ruleAvoidAreas.hpp"

#include "int/stores/fieldDimensionsStore.hpp"
#include "int/stores/robotStore.hpp"
#include "int/utilities/trace.hpp"

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
