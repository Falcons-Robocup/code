// Copyright 2015 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Author: jfeitsma
 * Creation: 2014-04-29
 *
 * Utility class: Circle.
 */

#ifndef CIRCLE_HPP
#define CIRCLE_HPP

#include "vector2d.hpp"

class Circle
{
    public:

        Vector2D pos;
        double   r;

        Circle() {}

        Circle(double xx, double yy, double rr)
        {
            pos.x = xx;
            pos.y = yy;
            r = rr;
        }


};



#endif

