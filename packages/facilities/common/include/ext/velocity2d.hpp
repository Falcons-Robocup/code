 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
    Velocity2D () : Vector2D()
{
        m_phi = 0.0;
};

    Velocity2D (double x, double y, double phi) : Vector2D(x, y)
    {
        m_phi = phi;
    };

    ~Velocity2D ()
    {
        // no resource release required
    };

    Velocity2D (const Velocity2D &other) : Vector2D(other)
    {
        m_phi = other.m_phi;
    }

    // operations
    Velocity2D& operator= (const Velocity2D & other)
    {
        if (other != *this)
        {
            x = other.x;
            y = other.y;
            m_phi = other.m_phi;
        }
        return *this;
    }

    Velocity2D& accelerate (double dx, double dy, double dphi)
    {
        this->x += dx;
        this->y += dy;
        this->m_phi += dphi;
        return *this;
    }

    // transformations
    Velocity2D& transformFCS2RCS (const Pose2D& robotpos)
    {
        // first translate, then rotate
        double angle = (M_PI/2 - robotpos.getPhi());
        Vector2D xynew = Vector2D(x, y).rotate(angle);
        x = xynew.x;
        y = xynew.y;
        // do not update angular velocity
        return *this;
    }

    Velocity2D& transformRCS2FCS (const Pose2D& robotpos)
    {
        // first rotate, then translate
        double angle = -(M_PI/2 - robotpos.getPhi());
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

    double getPhi () const
    {
        return m_phi;
    }


private:
    double m_phi;
};

}

#endif /* INCLUDED_VELOCITY2D_HPP */
