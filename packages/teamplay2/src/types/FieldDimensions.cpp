// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * FieldDimensions.cpp
 *
 *  Created on: Sep 6, 2016
 *      Author: Coen Tempelaars
 */
#include <stdexcept>

#include "cEnvironmentField.hpp"

#include "int/types/FieldDimensions.hpp"

using namespace teamplay;


const static float generic_margin = 0.1;
const static float wide_margin = 0.5;
const static float no_margin = 0.0;


areaName getAreaName (const FieldArea& area)
{
    areaName area_name = A_FIELD;

    switch (area)
    {
    case FieldArea::OPP_GOALAREA:
        area_name = A_OPP_GOALAREA_EXTENDED; // extend the area to avoid hitting the goal
        break;
    case FieldArea::OPP_PENALTYAREA:
        area_name = A_OPP_PENALTYAREA;
        break;
    case FieldArea::OPP_SIDE:
        area_name = A_OPP_SIDE;
        break;
    case FieldArea::OWN_GOALAREA:
        area_name = A_OWN_GOALAREA_EXTENDED; // extend the area to avoid hitting the goal
        break;
    case FieldArea::OWN_PENALTYAREA:
        area_name = A_OWN_PENALTYAREA;
        break;
    case FieldArea::OWN_SIDE:
        area_name = A_OWN_SIDE;
        break;
    default:
        throw std::runtime_error("FieldDimensions::getArea: input parameter area has an illegal value");
        break;
    }

    return area_name;
}


FieldDimensions::FieldDimensions()
{

}

FieldDimensions::~FieldDimensions()
{

}

float FieldDimensions::getWidth() const
{
    return cEnvironmentField::getInstance().getWidth();
}

float FieldDimensions::getLength() const
{
    return cEnvironmentField::getInstance().getLength();
}

Point2D FieldDimensions::getLocation(const FieldPOI poi) const
{
    poiName poi_name = P_CENTER;
    poiInfo poi_info = poiInfo();

    switch (poi)
    {
    case FieldPOI::OPP_GOALLINE_CENTER:
        poi_name = P_OPP_GOALLINE_CENTER;
        break;
    case FieldPOI::OPP_GOALPOST_LEFT:
        poi_name = P_OPP_GOALPOST_LEFT;
        break;
    case FieldPOI::OPP_GOALPOST_RIGHT:
        poi_name = P_OPP_GOALPOST_RIGHT;
        break;
    case FieldPOI::OWN_GOALLINE_CENTER:
        poi_name = P_OWN_GOALLINE_CENTER;
        break;
    case FieldPOI::OWN_GOALPOST_LEFT:
        poi_name = P_OWN_GOALPOST_LEFT;
        break;
    case FieldPOI::OWN_GOALPOST_RIGHT:
        poi_name = P_OWN_GOALPOST_RIGHT;
        break;
    case FieldPOI::OWN_GOALPOST_LEFTBACK:
        poi_name = P_OWN_GOALPOST_LEFTBACK;
        break;
    case FieldPOI::OWN_GOALPOST_RIGHTBACK:
        poi_name = P_OWN_GOALPOST_RIGHTBACK;
        break;
    case FieldPOI::OPP_GOALPOST_LEFTBACK:
        poi_name = P_OPP_GOALPOST_LEFTBACK;
        break;
    case FieldPOI::OPP_GOALPOST_RIGHTBACK:
        poi_name = P_OPP_GOALPOST_RIGHTBACK;
        break;
    case FieldPOI::OPP_GOALAREA_CORNER_RIGHT:
        poi_name = P_OPP_GOALAREA_CORNER_RIGHT;
        break;
    case FieldPOI::OWN_PENALTYAREA_CORNER_LEFT:
        poi_name = P_OWN_PENALTYAREA_CORNER_LEFT;
        break;
    case FieldPOI::OPP_PENALTYAREA_CORNER_LEFT:
        poi_name = P_OPP_PENALTYAREA_CORNER_LEFT;
        break;
    case FieldPOI::CENTER:
        poi_name = P_CENTER;
        break;
    case FieldPOI::CENTER_LEFT:
        poi_name = P_CENTER_LEFT;
        break;
    case FieldPOI::CENTER_RIGHT:
        poi_name = P_CENTER_RIGHT;
        break;
    case FieldPOI::TIP_IN:
        poi_name = P_TIP_IN;
        break;
    default:
        throw std::runtime_error("FieldDimensions::getLocation: input parameter poi has an illegal value");
        break;
    }

    cEnvironmentField::getInstance().getFieldPOI(poi_name, poi_info);
    return Point2D(poi_info.x, poi_info.y);
}

Point2D FieldDimensions::getLocation(const std::string poi) const
{
    poiInfo poi_info;
    cEnvironmentField::getInstance().getFieldPOIByString(poi, poi_info);
    return Point2D(poi_info.x, poi_info.y);
}

bool FieldDimensions::isValidPOI(const std::string poi) const
{
    poiInfo poi_info;
    return cEnvironmentField::getInstance().getFieldPOIByString(poi, poi_info);
}

Area2D FieldDimensions::getArea(const FieldArea area) const
{
    areaName area_name = getAreaName(area);
    areaInfo area_info = areaInfo();

    cEnvironmentField::getInstance().getFieldArea(area_name, area_info);
    return Area2D(area_info.R.getMinX(), area_info.R.getMinY(), area_info.R.getMaxX(), area_info.R.getMaxY());
}

Area2D FieldDimensions::getArea(const std::string area) const
{
    areaInfo area_info = areaInfo();
    cEnvironmentField::getInstance().getFieldAreaByString(area, area_info);
    return Area2D(area_info.R.getMinX(), area_info.R.getMinY(), area_info.R.getMaxX(), area_info.R.getMaxY());
}

bool FieldDimensions::isValidArea(const std::string area) const
{
    areaInfo area_info = areaInfo();
    return cEnvironmentField::getInstance().getFieldAreaByString(area, area_info);
}

bool FieldDimensions::isPositionInArea(const Point2D& position, const FieldArea& area) const
{
    auto area_name = getAreaName(area);
    return cEnvironmentField::getInstance().isPositionInArea(position.x, position.y, area_name, generic_margin);
}

bool FieldDimensions::isPositionNearArea(const Point2D& position, const FieldArea& area) const
{
    auto area_name = getAreaName(area);
    return cEnvironmentField::getInstance().isPositionInArea(position.x, position.y, area_name, wide_margin);
}

bool FieldDimensions::isPositionInField(const float x, const float y) const
{
    return cEnvironmentField::getInstance().isPositionInArea(x, y, A_FIELD, generic_margin);
}

bool FieldDimensions::isPositionInLeftSide(const float x, const float y) const
{
    return cEnvironmentField::getInstance().isPositionInArea(x, y, A_FIELD_LEFT, generic_margin);
}

bool FieldDimensions::isPositionInRightSide(const float x, const float y) const
{
    return cEnvironmentField::getInstance().isPositionInArea(x, y, A_FIELD_RIGHT, generic_margin);
}

bool FieldDimensions::isPositionInOwnSide(const float x, const float y) const
{
    return cEnvironmentField::getInstance().isPositionInArea(x, y, A_OWN_SIDE, generic_margin);
}

bool FieldDimensions::isPositionInOpponentSide(const float x, const float y) const
{
    return cEnvironmentField::getInstance().isPositionInArea(x, y, A_OPP_SIDE, generic_margin);
}

bool FieldDimensions::isPositionInOwnGoalArea(const float x, const float y) const
{
    return cEnvironmentField::getInstance().isPositionInArea(x, y, A_OWN_GOALAREA, generic_margin);
}

bool FieldDimensions::isPositionInOpponentGoalArea(const float x, const float y) const
{
    return cEnvironmentField::getInstance().isPositionInArea(x, y, A_OPP_GOALAREA, generic_margin);
}

bool FieldDimensions::isPositionInOwnPenaltyArea(const float x, const float y) const
{
    return cEnvironmentField::getInstance().isPositionInArea(x, y, A_OWN_PENALTYAREA, generic_margin);
}

bool FieldDimensions::isPositionInOpponentPenaltyArea(const float x, const float y) const
{
    return cEnvironmentField::getInstance().isPositionInArea(x, y, A_OPP_PENALTYAREA, generic_margin);
}

bool FieldDimensions::isPositionInSafetyBoundaries(const float x, const float y) const
{
    return cEnvironmentField::getInstance().isPositionInArea(x, y, A_FIELD_SAFETY_BOUNDARIES, no_margin);
}
