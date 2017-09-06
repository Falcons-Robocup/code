 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cObjectPath.hpp
 *
 *  Created on: Feb 18, 2016
 *      Author: Michel Koenen
 */

#ifndef COBJECTPATH_HPP_
#define COBJECTPATH_HPP_

#include "cTeamplayCommon.hpp"

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
