// Copyright 2018-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * obstacle.cpp
 *
 *  Created on: Jan 5, 2018
 *      Author: Coen Tempelaars
 */

#include "falconsCommon.hpp"

#include "int/types/obstacle.hpp"

using namespace teamplay;


obstacle::obstacle()
    : _position()
    , _velocity()
{ }

obstacle::obstacle(const Position2D& pos)
: _position(pos)
, _velocity()
{ }

obstacle::obstacle(const Position2D& pos, const Velocity2D& vel)
    : _position(pos)
    , _velocity(vel)
{ }

obstacle::~obstacle()
{ }

Point2D obstacle::getLocation() const
{
    return Point2D(_position.x, _position.y);
}

Position2D obstacle::getPosition() const
{
    return _position;
}

Velocity2D obstacle::getVelocity() const
{
    return _velocity;
}

double obstacle::getDistanceTo (const Point2D& p) const
{
    return calc_distance(getLocation(), p);
}

void obstacle::setPosition(const Position2D& position)
{
    _position = position;
}

void obstacle::setVelocity(const Velocity2D& velocity)
{
    _velocity = velocity;
}

