 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * fieldDimensions.cpp
 *
 *  Created on: Sep 6, 2016
 *      Author: Coen Tempelaars
 */
#include <stdexcept>

#include "cEnvironmentField.hpp"

#include "int/types/fieldDimensions.hpp"

using namespace teamplay;


const static float generic_margin = 0.1;
const static float wide_margin = 0.5;
const static float no_margin = 0.0;


areaName getAreaName (const fieldArea& area)
{
    areaName area_name = A_FIELD;

    switch (area)
    {
    case fieldArea::OPP_GOALAREA:
        area_name = A_OPP_GOALAREA_EXTENDED; // extend the area to avoid hitting the goal
        break;
    case fieldArea::OPP_PENALTYAREA:
        area_name = A_OPP_PENALTYAREA;
        break;
    case fieldArea::OPP_SIDE:
        area_name = A_OPP_SIDE;
        break;
    case fieldArea::OWN_GOALAREA:
        area_name = A_OWN_GOALAREA_EXTENDED; // extend the area to avoid hitting the goal
        break;
    case fieldArea::OWN_PENALTYAREA:
        area_name = A_OWN_PENALTYAREA;
        break;
    case fieldArea::OWN_SIDE:
        area_name = A_OWN_SIDE;
        break;
    default:
        throw std::runtime_error("fieldDimensions::getArea: input parameter area has an illegal value");
        break;
    }

    return area_name;
}


fieldDimensions::fieldDimensions()
{

}

fieldDimensions::~fieldDimensions()
{

}

float fieldDimensions::getWidth() const
{
    return cEnvironmentField::getInstance().getWidth();
}

float fieldDimensions::getLength() const
{
    return cEnvironmentField::getInstance().getLength();
}

Point2D fieldDimensions::getLocation(const fieldPOI poi) const
{
    poiName poi_name = P_CENTER;
    poiInfo poi_info = poiInfo();

    switch (poi)
    {
    case fieldPOI::OPP_GOALLINE_CENTER:
        poi_name = P_OPP_GOALLINE_CENTER;
        break;
    case fieldPOI::OPP_GOALPOST_LEFT:
        poi_name = P_OPP_GOALPOST_LEFT;
        break;
    case fieldPOI::OPP_GOALPOST_RIGHT:
        poi_name = P_OPP_GOALPOST_RIGHT;
        break;
    case fieldPOI::OWN_GOALLINE_CENTER:
        poi_name = P_OWN_GOALLINE_CENTER;
        break;
    case fieldPOI::OWN_GOALPOST_LEFT:
        poi_name = P_OWN_GOALPOST_LEFT;
        break;
    case fieldPOI::OWN_GOALPOST_RIGHT:
        poi_name = P_OWN_GOALPOST_RIGHT;
        break;
    case fieldPOI::OWN_GOALPOST_LEFTBACK:
        poi_name = P_OWN_GOALPOST_LEFTBACK;
        break;
    case fieldPOI::OWN_GOALPOST_RIGHTBACK:
        poi_name = P_OWN_GOALPOST_RIGHTBACK;
        break;
    case fieldPOI::OPP_GOALPOST_LEFTBACK:
        poi_name = P_OPP_GOALPOST_LEFTBACK;
        break;
    case fieldPOI::OPP_GOALPOST_RIGHTBACK:
        poi_name = P_OPP_GOALPOST_RIGHTBACK;
        break;
    case fieldPOI::OPP_GOALAREA_CORNER_RIGHT:
        poi_name = P_OPP_GOALAREA_CORNER_RIGHT;
        break;
    case fieldPOI::OWN_PENALTYAREA_CORNER_LEFT:
        poi_name = P_OWN_PENALTYAREA_CORNER_LEFT;
        break;
    case fieldPOI::OPP_PENALTYAREA_CORNER_LEFT:
        poi_name = P_OPP_PENALTYAREA_CORNER_LEFT;
        break;
    case fieldPOI::CENTER:
        poi_name = P_CENTER;
        break;
    case fieldPOI::CENTER_LEFT:
        poi_name = P_CENTER_LEFT;
        break;
    case fieldPOI::CENTER_RIGHT:
        poi_name = P_CENTER_RIGHT;
        break;
    case fieldPOI::TIP_IN:
        poi_name = P_TIP_IN;
        break;
    default:
        throw std::runtime_error("fieldDimensions::getLocation: input parameter poi has an illegal value");
        break;
    }

    cEnvironmentField::getInstance().getFieldPOI(poi_name, poi_info);
    return Point2D(poi_info.x, poi_info.y);
}

Point2D fieldDimensions::getLocation(const std::string poi) const
{
    poiInfo poi_info;
    cEnvironmentField::getInstance().getFieldPOIByString(poi, poi_info);
    return Point2D(poi_info.x, poi_info.y);
}

bool fieldDimensions::isValidPOI(const std::string poi) const
{
    poiInfo poi_info;
    return cEnvironmentField::getInstance().getFieldPOIByString(poi, poi_info);
}

Area2D fieldDimensions::getArea(const fieldArea area) const
{
    areaName area_name = getAreaName(area);
    areaInfo area_info = areaInfo();

    cEnvironmentField::getInstance().getFieldArea(area_name, area_info);
    return Area2D(area_info.R.getMinX(), area_info.R.getMinY(), area_info.R.getMaxX(), area_info.R.getMaxY());
}

Area2D fieldDimensions::getArea(const std::string area) const
{
    areaInfo area_info = areaInfo();
    cEnvironmentField::getInstance().getFieldAreaByString(area, area_info);
    return Area2D(area_info.R.getMinX(), area_info.R.getMinY(), area_info.R.getMaxX(), area_info.R.getMaxY());
}

bool fieldDimensions::isValidArea(const std::string area) const
{
    areaInfo area_info = areaInfo();
    return cEnvironmentField::getInstance().getFieldAreaByString(area, area_info);
}

bool fieldDimensions::isPositionInArea(const Position2D& position, const fieldArea& area) const
{
    auto area_name = getAreaName(area);
    return cEnvironmentField::getInstance().isPositionInArea(position.x, position.y, area_name, generic_margin);
}

bool fieldDimensions::isPositionNearArea(const Position2D& position, const fieldArea& area) const
{
    auto area_name = getAreaName(area);
    return cEnvironmentField::getInstance().isPositionInArea(position.x, position.y, area_name, wide_margin);
}

bool fieldDimensions::isPositionInField(const float x, const float y) const
{
    return cEnvironmentField::getInstance().isPositionInArea(x, y, A_FIELD, generic_margin);
}

bool fieldDimensions::isPositionInLeftSide(const float x, const float y) const
{
    return cEnvironmentField::getInstance().isPositionInArea(x, y, A_FIELD_LEFT, generic_margin);
}

bool fieldDimensions::isPositionInRightSide(const float x, const float y) const
{
    return cEnvironmentField::getInstance().isPositionInArea(x, y, A_FIELD_RIGHT, generic_margin);
}

bool fieldDimensions::isPositionInOwnSide(const float x, const float y) const
{
    return cEnvironmentField::getInstance().isPositionInArea(x, y, A_OWN_SIDE, generic_margin);
}

bool fieldDimensions::isPositionInOpponentSide(const float x, const float y) const
{
    return cEnvironmentField::getInstance().isPositionInArea(x, y, A_OPP_SIDE, generic_margin);
}

bool fieldDimensions::isPositionInOwnGoalArea(const float x, const float y) const
{
    return cEnvironmentField::getInstance().isPositionInArea(x, y, A_OWN_GOALAREA, generic_margin);
}

bool fieldDimensions::isPositionInOpponentGoalArea(const float x, const float y) const
{
    return cEnvironmentField::getInstance().isPositionInArea(x, y, A_OPP_GOALAREA, generic_margin);
}

bool fieldDimensions::isPositionInOwnPenaltyArea(const float x, const float y) const
{
    return cEnvironmentField::getInstance().isPositionInArea(x, y, A_OWN_PENALTYAREA, generic_margin);
}

bool fieldDimensions::isPositionInOpponentPenaltyArea(const float x, const float y) const
{
    return cEnvironmentField::getInstance().isPositionInArea(x, y, A_OPP_PENALTYAREA, generic_margin);
}

bool fieldDimensions::isPositionInSafetyBoundaries(const float x, const float y) const
{
    return cEnvironmentField::getInstance().isPositionInArea(x, y, A_FIELD_SAFETY_BOUNDARIES, no_margin);
}
