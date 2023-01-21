// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * FieldDimensions.hpp
 *
 *  Created on: Aug 30, 2016
 *      Author: Coen Tempelaars
 */

#ifndef FIELDDIMENSIONS_HPP_
#define FIELDDIMENSIONS_HPP_

#include <string>

#include "area2D.hpp"
#include "vector2d.hpp"

namespace teamplay
{

enum class FieldPOI {
    OPP_GOALLINE_CENTER,
    OPP_GOALPOST_LEFT,
    OPP_GOALPOST_RIGHT,
    OWN_GOALLINE_CENTER,
    OWN_GOALPOST_LEFT,
    OWN_GOALPOST_RIGHT,
    OWN_GOALPOST_LEFTBACK,
    OWN_GOALPOST_RIGHTBACK,
    OPP_GOALPOST_LEFTBACK,
    OPP_GOALPOST_RIGHTBACK,
    OPP_GOALAREA_CORNER_RIGHT,
    OWN_PENALTYAREA_CORNER_LEFT,
    OPP_PENALTYAREA_CORNER_LEFT,
    CENTER,
    CENTER_LEFT,
    CENTER_RIGHT,
    TIP_IN
};

enum class FieldArea {
    OPP_GOALAREA,
    OPP_PENALTYAREA,
    OPP_SIDE,
    OWN_GOALAREA,
    OWN_PENALTYAREA,
    OWN_SIDE
};

class FieldDimensions {
public:
    FieldDimensions ();
    virtual ~FieldDimensions();

    virtual float getWidth() const;
    virtual float getLength() const;

    virtual Point2D getLocation(const FieldPOI poi) const;
    virtual Point2D getLocation(const std::string poi) const;

    virtual Area2D getArea(const FieldArea area) const;
    virtual Area2D getArea(const std::string area) const;

    virtual bool isValidPOI(const std::string poi) const;
    virtual bool isValidArea(const std::string area) const;

    virtual bool isPositionInArea(const Point2D&, const FieldArea&) const;
    virtual bool isPositionNearArea(const Point2D&, const FieldArea&) const;

    virtual bool isPositionInField(const float x, const float y) const;
    virtual bool isPositionInLeftSide(const float x, const float y) const;
    virtual bool isPositionInRightSide(const float x, const float y) const;
    virtual bool isPositionInOwnSide(const float x, const float y) const;
    virtual bool isPositionInOpponentSide(const float x, const float y) const;
    virtual bool isPositionInOwnGoalArea(const float x, const float y) const;
    virtual bool isPositionInOpponentGoalArea(const float x, const float y) const;
    virtual bool isPositionInOwnPenaltyArea(const float x, const float y) const;
    virtual bool isPositionInOpponentPenaltyArea(const float x, const float y) const;
    virtual bool isPositionInSafetyBoundaries(const float x, const float y) const;


private:

};

} /* namespace teamplay */

#endif /* FIELDDIMENSIONS_HPP_ */
