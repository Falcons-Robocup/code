// Copyright 2018 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * obstacle.hpp
 *
 *  Created on: Jan 5, 2018
 *      Author: Coen Tempelaars
 */

#ifndef OBSTACLE_HPP_
#define OBSTACLE_HPP_

#include "position2d.hpp"

namespace teamplay
{

class obstacle {
public:
    obstacle();
    obstacle(const Position2D&);
    obstacle(const Position2D&, const Velocity2D&);
    virtual ~obstacle();

    virtual Point2D getLocation() const;
    virtual Position2D getPosition() const;
    virtual Velocity2D getVelocity() const;
    virtual double getDistanceTo (const Point2D&) const;

    virtual void setPosition(const Position2D&);
    virtual void setVelocity(const Velocity2D&);

private:
    Position2D _position;
    Velocity2D _velocity;
};

} /* namespace teamplay */

#endif /* OBSTACLE_HPP_ */
