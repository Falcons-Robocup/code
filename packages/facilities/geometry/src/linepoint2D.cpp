// Copyright 2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * linepoint2D.cpp
 *
 *  Created on: Jul 23, 2017
 *      Author: Tim Kouters
 */

#include "ext/linepoint2D.hpp"

linepoint2D::linepoint2D()
{
	_source = Point2D();
	_destination = Point2D();
}

linepoint2D::linepoint2D(Point2D source, Point2D destination)
{
	_source = source;
	_destination = destination;
}

linepoint2D::linepoint2D(float sourceX, float sourceY, float destinationX, float destinationY)
{
	_source = Point2D(sourceX, sourceY);
	_destination = Point2D(destinationX, destinationY);
}

linepoint2D::~linepoint2D()
{

}

Point2D linepoint2D::getSourcePoint2D()
{
	return _source;
}

Vector2D linepoint2D::getSourceVector2D()
{
	return Vector2D(_source.x, _source.y);
}

Point2D linepoint2D::getDestinationPoint2D()
{
	return _destination;
}

Vector2D linepoint2D::getDestinationVector2D()
{
	return Vector2D(_destination.x, _destination.y);
}
