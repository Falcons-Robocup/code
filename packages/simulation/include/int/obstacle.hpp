// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * obstacle.hpp
 *
 *  Created on: November, 2019
 *      Author: Jan Feitsma
 */

#ifndef OBSTACLE_HPP_
#define OBSTACLE_HPP_

#include "falconsCommon.hpp" // TODO #14


struct Obstacle
{
    Vector2D position;
    Vector2D velocity;
    float radius;
    Obstacle(Circle c)
    {
        position.x = c.pos.x;
        position.y = c.pos.y;
        radius = c.r;
        velocity.x = 0.0;
        velocity.y = 0.0;
    }
    Obstacle(Circle c, Vector2D s)
    {
        position.x = c.pos.x;
        position.y = c.pos.y;
        radius = c.r;
        velocity.x = s.x;
        velocity.y = s.y;
    }
    Circle asCircle() const
    {
        return Circle(position.x, position.y, radius);
    }
};

#endif /* BALL_HPP_ */
