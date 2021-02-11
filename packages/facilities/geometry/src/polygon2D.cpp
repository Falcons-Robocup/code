// Copyright 2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * polygon2D.cpp
 *
 *  Created on: Jul 23, 2017
 *      Author: Tim Kouters
 */

#include "ext/polygon2D.hpp"

#include <stddef.h>

polygon2D::polygon2D()
{
	_points.clear();
}

polygon2D::polygon2D(std::vector<Point2D> points)
{
	_points = points;
}

polygon2D::~polygon2D()
{

}

polygon2D::polygon2D(Area2D area)
{
	Point2D ll;
	Point2D lr;
	Point2D ul;
	Point2D ur;

	ll.x = area.ll.x;
	ll.y = area.ll.y;
	lr.x = area.lr.x;
	lr.y = area.lr.y;
	ul.x = area.ul.x;
	ul.y = area.ul.y;
	ur.x = area.ur.x;
	ur.y = area.ur.y;

	_points.push_back(ll);
	_points.push_back(ul);
	_points.push_back(ur);
	_points.push_back(lr);
}

void polygon2D::addPoint(const Point2D point)
{
	_points.push_back(point);
}

void polygon2D::addPoint(float x, float y)
{
	Point2D point(x, y);
	_points.push_back(point);
}

void polygon2D::addPoints(std::vector<Point2D> points)
{
	_points.insert(_points.end(), points.begin(), points.end());
}

bool polygon2D::pointExistsInPolygon(Point2D point)
{
	std::vector<Point2D> points = getPoints();
	size_t i, j, nvert = points.size();
	bool retVal = false;

	for(i = 0, j = nvert - 1; i < nvert; j = i++)
	{
		if(((points[i].y >= point.y ) != (points[j].y >= point.y) ) &&
			(point.x <= (points[j].x - points[i].x) * (point.y - points[i].y) / (points[j].y - points[i].y) + points[i].x)
		  )
		  retVal = !retVal;
	  }

	return retVal;
}

std::vector<Point2D> polygon2D::getPoints() const
{
	return _points;
}

std::vector<linepoint2D> polygon2D::getLinepoints()
{
	std::vector<linepoint2D> linepoints;

	for(size_t i = 0; i < _points.size()-1; i++)
	{
		linepoint2D line(_points.at(i), _points.at(i+1));
		linepoints.push_back(line);
	}

	if(_points.size() > 1)
	{
		linepoint2D line(_points.at(0), _points.at(_points.size()-1));
		linepoints.push_back(line);
	}

	return linepoints;
}
