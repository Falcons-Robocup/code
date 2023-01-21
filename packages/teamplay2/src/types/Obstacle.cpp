// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Obstacle.cpp
 *
 *  Created on: Jan 5, 2018
 *      Author: Coen Tempelaars
 */

#include "falconsCommon.hpp"

#include "int/types/Obstacle.hpp"

using namespace teamplay;


Obstacle::Obstacle()
    : _position()
    , _velocity()
{ }

Obstacle::Obstacle(const geometry::Pose2D& pos)
: _position(pos)
, _velocity()
{ }

Obstacle::Obstacle(const geometry::Pose2D& pos, const geometry::Velocity2D& vel)
    : _position(pos)
    , _velocity(vel)
{ }

Obstacle::~Obstacle()
{ }

Point2D Obstacle::getLocation() const
{
    return Point2D(_position.x, _position.y);
}

geometry::Pose2D Obstacle::getPosition() const
{
    return _position;
}

geometry::Velocity2D Obstacle::getVelocity() const
{
    return _velocity;
}

double Obstacle::getDistanceTo (const Point2D& p) const
{
    return calc_distance(getLocation(), p);
}

void Obstacle::setPosition(const geometry::Pose2D& position)
{
    _position = position;
}

void Obstacle::setVelocity(const geometry::Velocity2D& velocity)
{
    _velocity = velocity;
}

