// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Obstacle.hpp
 *
 *  Created on: Jan 5, 2018
 *      Author: Coen Tempelaars
 */

#ifndef OBSTACLE_HPP_
#define OBSTACLE_HPP_

#include "pose2d.hpp"
#include "velocity2d.hpp"

namespace teamplay
{

class Obstacle {
public:
    Obstacle();
    Obstacle(const geometry::Pose2D&);
    Obstacle(const geometry::Pose2D&, const geometry::Velocity2D&);
    virtual ~Obstacle();

    virtual Point2D getLocation() const;
    virtual geometry::Pose2D getPosition() const;
    virtual geometry::Velocity2D getVelocity() const;
    virtual double getDistanceTo (const Point2D&) const;

    virtual void setPosition(const geometry::Pose2D&);
    virtual void setVelocity(const geometry::Velocity2D&);

private:
    geometry::Pose2D _position;
    geometry::Velocity2D _velocity;
};

} /* namespace teamplay */

#endif /* OBSTACLE_HPP_ */
