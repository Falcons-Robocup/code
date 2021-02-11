// Copyright 2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * linepoint2D.hpp
 *
 *  Created on: Jul 23, 2017
 *      Author: Tim Kouters
 */

#ifndef LINEPOINT2D_HPP_
#define LINEPOINT2D_HPP_

#include <vector2d.hpp>

class linepoint2D
{
	public:
		linepoint2D();
		linepoint2D(Point2D source, Point2D destination);
		linepoint2D(float sourceX, float sourceY, float destinationX, float destinationY);
		~linepoint2D();

		Point2D getSourcePoint2D();
		Vector2D getSourceVector2D();

		Point2D getDestinationPoint2D();
		Vector2D getDestinationVector2D();

	private:
		Point2D _source;
		Point2D _destination;
};

#endif /* LINEPOINT2D_HPP_ */
