// Copyright 2015 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
