// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Ball.cpp
 *
 *  Created on: Aug 11, 2016
 *      Author: Coen Tempelaars
 */

#include "int/types/Ball.hpp"

#include "int/stores/FieldDimensionsStore.hpp"

using namespace teamplay;


Ball::Ball()
{
    reset();
}

Ball::~Ball() { }

void Ball::reset()
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

bool Ball::isLocationKnown() const
{
    return _isCurrentPositionKnown;
}

bool Ball::isVelocityKnown() const
{
    return _isCurrentVelocityKnown;
}

bool Ball::isInsideField() const
{
    return FieldDimensionsStore::getFieldDimensions().isPositionInField
            (_currentPosition.x, _currentPosition.y);
}

bool Ball::isAtOwnSide() const
{
    return FieldDimensionsStore::getFieldDimensions().isPositionInOwnSide
            (_currentPosition.x, _currentPosition.y);
}

bool Ball::isAtOpponentSide() const
{
    return FieldDimensionsStore::getFieldDimensions().isPositionInOpponentSide
            (_currentPosition.x, _currentPosition.y);
}

bool Ball::isAtLeftSide() const
{
    return FieldDimensionsStore::getFieldDimensions().isPositionInLeftSide
            (_currentPosition.x, _currentPosition.y);
}

bool Ball::isAtRightSide() const
{
    return FieldDimensionsStore::getFieldDimensions().isPositionInRightSide
            (_currentPosition.x, _currentPosition.y);
}

bool Ball::isInOwnPenaltyArea() const
{
    return FieldDimensionsStore::getFieldDimensions().isPositionInOwnPenaltyArea
            (_currentPosition.x, _currentPosition.y);
}

bool Ball::isInOpponentPenaltyArea() const
{
    return FieldDimensionsStore::getFieldDimensions().isPositionInOpponentPenaltyArea
            (_currentPosition.x, _currentPosition.y);
}

bool Ball::isClaimedOnOpponentHalf() const
{
    return FieldDimensionsStore::getFieldDimensions().isPositionInOpponentSide
            (_claimedPosition.x, _claimedPosition.y);
}

bool Ball::mustBeAvoided() const
{
    return _mustBeAvoided;
}

Point2D Ball::getLocation() const
{
    return Point2D(_currentPosition.x, _currentPosition.y);
}

Point3D Ball::getPosition() const
{
    return _currentPosition;
}

Vector3D Ball::getVelocity() const
{
    return _currentVelocity;
}

Point2D Ball::getClaimedLocation() const
{
    return Point2D(_claimedPosition.x, _claimedPosition.y);
}

Point3D Ball::getClaimedPosition() const
{
    return _claimedPosition;
}

void Ball::setPosition(const Point3D& p)
{
    _isCurrentPositionKnown = true;
    _currentPosition = p;
}

void Ball::setVelocity(const Vector3D& v)
{
    _isCurrentVelocityKnown = true;
    _currentVelocity = v;
}

void Ball::setPositionUnknown()
{
    _isCurrentPositionKnown = false;
}

void Ball::setVelocityUnknown()
{
    _isCurrentVelocityKnown = false;
}

void Ball::setPositionClaimed(const Point3D& p)
{
    _isClaimedPositionKnown = true;
    _claimedPosition = p;
}

void Ball::setPositionClaimedUnknown()
{
    _isClaimedPositionKnown = false;
}

void Ball::avoid()
{
    _mustBeAvoided = true;
}
