// Copyright 2016-2020 Michel Koenen (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cObjectPath.hpp
 *
 *  Created on: Feb 18, 2016
 *      Author: Michel Koenen
 */

#ifndef COBJECTPATH_HPP_
#define COBJECTPATH_HPP_

#include "falconsCommon.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

//#include "int/types/cPositionTypes.hpp"
//#include "int/types/cRobotLocationTypes.hpp"
//#include "int/types/cBallLocationTypes.hpp"


typedef boost::geometry::model::point<float, 2, boost::geometry::cs::cartesian> Point;
typedef boost::geometry::model::segment<Point> Segment;

class cObjectPath
{
	public:
		cObjectPath(Point3D &location, Vector3D &speedvector);
		bool intersectCheck( Point2D lineStart, Point2D lineEnd , Point2D &intersectCoord);
		bool intersectCheck( Point2D lineStart, Point2D lineEnd );
		Point2D getPerpendicularInterceptionPoint( Point2D otherObject, bool &valid);
		float calcY(float forX);
		float calcX(float forY);
		float getSlope();
		float getYIntercept();
		Point2D getTowardsCoordinate();
		float getX();
		float getY();
		float getSpeedX();
		float getSpeedY();

	private:
		float slope;
		float intercept;
		Point2D myPosition;
		Point2D mySpeed;
};

#endif /* COBJECTPATH_HPP_ */
