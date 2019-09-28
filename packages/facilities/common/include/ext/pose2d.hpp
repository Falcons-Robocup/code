 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Author: ctempela
 * Creation: 2015-11-19
 *
 * Pose2D class: position and angle [0..2*pi] in the 2D plane
 * The entire class is implemented inline for performance reasons
 */

#ifndef INCLUDED_POSE2D_HPP
#define INCLUDED_POSE2D_HPP

#include <math.h>

#include "vector2d.hpp"

namespace geometry
{

class Pose2D : public Point2D
{
public:
	Pose2D () : Point2D()
    {
		m_phi = 0.0;
    };

	Pose2D (double x, double y, double phi) : Point2D(x, y)
    {
		m_phi = normalize(phi);
    };

	~Pose2D ()
	{
		// no resource release required
	};

	Pose2D (const Pose2D &other) : Point2D(other)
	{
		m_phi = other.m_phi;
	}

    // operations
	Pose2D& operator= (const Pose2D & other)
	{
		if (other != *this)
		{
			x = other.x;
			y = other.y;
			m_phi = other.m_phi;
		}
		return *this;
	}

	Pose2D& teleport (double x, double y, double phi)
	{
		this->x = x;
		this->y = y;
		this->m_phi = normalize(phi);
		return *this;
	}

	Pose2D& shift (double distance_in_x, double distance_in_y)
	{
		this->x += distance_in_x;
		this->y += distance_in_y;
		return *this;
	}

	Pose2D& turn (double rad)
	{
		m_phi = normalize(m_phi + rad);
		return *this;
	}

	// transformations
	Pose2D& transformFCS2RCS (const Pose2D& robotpos)
	{
	    // first translate, then rotate
		double angle = (M_PI/2 - robotpos.m_phi);
		Vector2D xynew = (Vector2D(x, y) - Vector2D(robotpos.x, robotpos.y)).rotate(angle);
		x = xynew.x;
		y = xynew.y;
		m_phi = normalize(m_phi + angle);
		return *this;
	}

	Pose2D& transformRCS2FCS (const Pose2D& robotpos)
	{
	    // first rotate, then translate
		double angle = - (M_PI/2 - robotpos.m_phi);
		Vector2D xyrot = (Vector2D(x, y)).rotate(angle);
		x = xyrot.x + robotpos.x;
		y = xyrot.y + robotpos.y;
		m_phi = normalize(m_phi + angle);
		return *this;
	}

	Pose2D& transformACS2FCS (bool playing_left_to_right)
	{
	    // no change in case we are playing left to right
	    if (!playing_left_to_right)
	    {
	        // rotate by half a circle
	        x = -x;
	        y = -y;
	    	m_phi = normalize(m_phi + M_PI);
	    }
		return *this;
	}

	Pose2D& transformFCS2ACS (bool playing_left_to_right)
	{
		// no change in case we are playing left to right
		if (!playing_left_to_right)
		{
			// rotate by half a circle
			x = -x;
			y = -y;
			m_phi = normalize(m_phi + M_PI);
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

	double normalize (double phi)
	{
		double normalized_phi = phi;
		while (normalized_phi < 0.0)      { normalized_phi += (2*M_PI); }
		while (normalized_phi > (2*M_PI)) { normalized_phi -= (2*M_PI); }
		return normalized_phi;
	}
};

}

#endif /* INCLUDED_POSE2D_HPP */
