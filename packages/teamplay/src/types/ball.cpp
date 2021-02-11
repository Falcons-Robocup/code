// Copyright 2016-2018 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ball.cpp
 *
 *  Created on: Aug 11, 2016
 *      Author: Coen Tempelaars
 */

#include "int/types/ball.hpp"

#include "int/stores/fieldDimensionsStore.hpp"

using namespace teamplay;


ball::ball()
{
    reset();
}

ball::~ball() { }

void ball::reset()
{
    /* We do not reset the current position and current velocity here.
     * The idea is that we always maintain the last known position and velocity
     * as the "current" position and velocity. Besides that, we maintain
     * whether we actually know the current position and velocity. */

    _isCurrentPositionKnown = false;
    _isCurrentVelocityKnown = false;
    _isClaimedPositionKnown = false;
    _claimedPosition = Point3D();
    _mustBeAvoided = false;
}

bool ball::isLocationKnown() const
{
    return _isCurrentPositionKnown;
}

bool ball::isVelocityKnown() const
{
    return _isCurrentVelocityKnown;
}

bool ball::isInsideField() const
{
    return fieldDimensionsStore::getFieldDimensions().isPositionInField
            (_currentPosition.x, _currentPosition.y);
}

bool ball::isAtOwnSide() const
{
    return fieldDimensionsStore::getFieldDimensions().isPositionInOwnSide
            (_currentPosition.x, _currentPosition.y);
}

bool ball::isAtOpponentSide() const
{
    return fieldDimensionsStore::getFieldDimensions().isPositionInOpponentSide
            (_currentPosition.x, _currentPosition.y);
}

bool ball::isAtLeftSide() const
{
    return fieldDimensionsStore::getFieldDimensions().isPositionInLeftSide
            (_currentPosition.x, _currentPosition.y);
}

bool ball::isAtRightSide() const
{
    return fieldDimensionsStore::getFieldDimensions().isPositionInRightSide
            (_currentPosition.x, _currentPosition.y);
}

bool ball::isInOwnPenaltyArea() const
{
    return fieldDimensionsStore::getFieldDimensions().isPositionInOwnPenaltyArea
            (_currentPosition.x, _currentPosition.y);
}

bool ball::isInOpponentPenaltyArea() const
{
    return fieldDimensionsStore::getFieldDimensions().isPositionInOpponentPenaltyArea
            (_currentPosition.x, _currentPosition.y);
}

bool ball::isClaimedOnOpponentHalf() const
{
    return fieldDimensionsStore::getFieldDimensions().isPositionInOpponentSide
            (_claimedPosition.x, _claimedPosition.y);
}

bool ball::mustBeAvoided() const
{
    return _mustBeAvoided;
}

Point2D ball::getLocation() const
{
    return Point2D(_currentPosition.x, _currentPosition.y);
}

Point3D ball::getPosition() const
{
    return _currentPosition;
}

Vector3D ball::getVelocity() const
{
    return _currentVelocity;
}

Point2D ball::getClaimedLocation() const
{
    return Point2D(_claimedPosition.x, _claimedPosition.y);
}

Point3D ball::getClaimedPosition() const
{
    return _claimedPosition;
}

void ball::setPosition(const Point3D& p)
{
    _isCurrentPositionKnown = true;
    _currentPosition = p;
}

void ball::setVelocity(const Vector3D& v)
{
    _isCurrentVelocityKnown = true;
    _currentVelocity = v;
}

void ball::setPositionUnknown()
{
    _isCurrentPositionKnown = false;
}

void ball::setVelocityUnknown()
{
    _isCurrentVelocityKnown = false;
}

void ball::setPositionClaimed(const Point3D& p)
{
    _isClaimedPositionKnown = true;
    _claimedPosition = p;
}

void ball::setPositionClaimedUnknown()
{
    _isClaimedPositionKnown = false;
}

void ball::avoid()
{
    _mustBeAvoided = true;
}
