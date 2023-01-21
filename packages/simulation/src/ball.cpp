// Copyright 2018-2022 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ball.cpp
 *
 *  Created on: Nov 21, 2018
 *      Author: Coen Tempelaars
 */

#include "int/ball.hpp"

#include "tracing.hpp"

const static float minimalMovingSpeed = 0.01;

Point2D Ball::getLocation() const
{
    return Point2D(_position.x, _position.y);
}

Point2D Ball::getPickupLocation() const
{
    return _pickupLocation;
}

Point3D Ball::getPosition() const
{
    return _position;
}

// Point3D Ball::getPositionFCS(const TeamID& teamID) const
// {
//     if (teamID == TeamID::A)
//     {
//         return _position;
//     }
//     else
//     {
//         return (_position * -1.0);
//     }
// }

float Ball::getSpeed() const
{
    return vectorsize(_velocity);
}

Vector3D Ball::getVelocity() const
{
    return _velocity;
}

bool Ball::isMoving() const
{
    if (vectorsize(_velocity) > minimalMovingSpeed)
    {
        TRACE("The ball is moving");
        return true;
    }
    else
    {
        TRACE("The ball is not moving");
        return false;
    }
}

void Ball::setLocation (const Point2D& p)
{
    this->setPosition(Point3D(p.x, p.y, 0.0));
}

void Ball::setPickupLocation (const Point2D& p)
{
    _pickupLocation = p;
}

void Ball::setPosition (const Point3D& p)
{
    _position = p;
}

void Ball::setVelocity (const Vector3D& v)
{
    _velocity = v;
}

void Ball::stopMoving()
{
    _velocity = Vector3D();
}

void Ball::teleport(const Vector2D& v)
{
    this->teleport(v.x, v.y);
}

void Ball::teleport (const float x, const float y)
{
    this->teleport(x, y, 0.0);
}

void Ball::teleport (const float x, const float y, const float z)
{
    _position = Point3D(x, y, z);
    _velocity = Vector3D();
}
