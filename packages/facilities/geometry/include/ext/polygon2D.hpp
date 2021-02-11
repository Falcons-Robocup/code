// Copyright 2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * polygon2D.hpp
 *
 *  Created on: Jul 23, 2017
 *      Author: Tim Kouters
 */

#ifndef POLYGON2D_HPP_
#define POLYGON2D_HPP_

#include <vector>

#include <vector2d.hpp>
#include <area2D.hpp>
#include <linepoint2D.hpp>

class polygon2D
{
	public:
		polygon2D();
		polygon2D(std::vector<Point2D> points);
		polygon2D(Area2D area);

		~polygon2D();

		void addPoint(const Point2D point);
		void addPoint(float x, float y);
		void addPoints(std::vector<Point2D> points);
		bool pointExistsInPolygon(Point2D point);

		std::vector<Point2D> getPoints() const;
		std::vector<linepoint2D> getLinepoints();

	private:
		std::vector<Point2D> _points;
};


#endif /* POLYGON2D_HPP_ */
