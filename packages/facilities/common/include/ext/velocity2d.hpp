// Copyright 2015-2021 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Author: ctempela
 * Creation: 2015-11-25
 *
 * Velocity2D class: velocity in the 2D plane: in x, in y and angular velocity
 * The entire class is implemented inline for performance reasons
 */

#ifndef INCLUDED_VELOCITY2D_HPP
#define INCLUDED_VELOCITY2D_HPP

#include <math.h>

#include "vector2d.hpp"
#include "pose2d.hpp"

namespace geometry
{

class Velocity2D : public Vector2D
{
public:

    double Rz;

    Velocity2D () : Vector2D()
    {
            Rz = 0.0;
    };

    Velocity2D (double x, double y, double phi) : Vector2D(x, y)
    {
        Rz = phi;
    };

    ~Velocity2D ()
    {
        // no resource release required
    };

    Velocity2D (const Velocity2D &other) : Vector2D(other)
    {
        Rz = other.Rz;
    }

    // operations
    Velocity2D& operator= (const Velocity2D & other)
    {
        if (other != *this)
        {
            x = other.x;
            y = other.y;
            Rz = other.Rz;
        }
        return *this;
    }

    Velocity2D& accelerate (double dx, double dy, double dphi)
    {
        this->x += dx;
        this->y += dy;
        this->Rz += dphi;
        return *this;
    }

    // transformations
    Velocity2D& transformFCS2RCS (const Pose2D& robotpos)
    {
        // first translate, then rotate
        double angle = (M_PI/2 - robotpos.Rz);
        Vector2D xynew = Vector2D(x, y).rotate(angle);
        x = xynew.x;
        y = xynew.y;
        // do not update angular velocity
        return *this;
    }

    Velocity2D& transformRCS2FCS (const Pose2D& robotpos)
    {
        // first rotate, then translate
        double angle = -(M_PI/2 - robotpos.Rz);
        Vector2D xynew = Vector2D(x, y).rotate(angle);
        x = xynew.x;
        y = xynew.y;
        // do not update angular velocity
        return *this;
    }

    Velocity2D& transformACS2FCS (bool playing_left_to_right)
    {
        // no change in case we are playing left to right
        if (!playing_left_to_right)
        {
            // rotate by half a circle
            x = -x;
            y = -y;
        }
        return *this;
    }

    Velocity2D& transformFCS2ACS (bool playing_left_to_right)
    {
        // no change in case we are playing left to right
        if (!playing_left_to_right)
        {
            // rotate by half a circle
            x = -x;
            y = -y;
        }
        return *this;
    }

    // getters
    double getX() const
    {
        return x;
    }

    double getY() const
    {
        return y;
    }

    double getRz () const
    {
        return Rz;
    }

};

}

#endif /* INCLUDED_VELOCITY2D_HPP */
