 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
