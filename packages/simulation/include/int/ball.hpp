// Copyright 2018-2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ball.hpp
 *
 *  Created on: Nov 21, 2018
 *      Author: Coen Tempelaars
 */

#ifndef BALL_HPP_
#define BALL_HPP_

#include "teamID.hpp"
#include "vector3d.hpp"

class Ball {
public:
    Point2D getLocation() const;
    Point2D getPickupLocation() const;
    Point3D getPosition() const;
    Point3D getPositionFCS(const TeamID&) const;
    float getSpeed() const;
    Vector3D getVelocity() const;
    bool isMoving() const;

    void setLocation (const Point2D&);
    void setPickupLocation (const Point2D&);
    void setPosition (const Point3D&);
    void setVelocity (const Vector3D&);
    void stopMoving();

    void teleport (const Vector2D&);
    void teleport (const float x, const float y);
    void teleport (const float x, const float y, const float z);

private:
    Point2D _pickupLocation;
    Point3D _position;
    Vector3D _velocity;
};

#endif /* BALL_HPP_ */
