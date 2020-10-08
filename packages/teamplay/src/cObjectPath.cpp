 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cObjectPath.cpp
 *
 *  Created on: Feb 18, 2015
 *      Author: Michel Koenen
 */

#include "int/cObjectPath.hpp"

#include <boost/geometry/algorithms/intersection.hpp>

#include "int/stores/fieldDimensionsStore.hpp"

using namespace teamplay;


cObjectPath::cObjectPath(Point3D &position, Vector3D &speed)
{
    myPosition.x=position.x;
    myPosition.y=position.y;
    mySpeed.x=speed.x;
    mySpeed.y=speed.y;

    // y= slope*x + intercept;
    // intercept= y - slope*x
    slope=mySpeed.y/mySpeed.x;
    intercept=myPosition.y - slope * myPosition.x;
}

float cObjectPath::getX()
{
	return myPosition.x;
}

float cObjectPath::getY()
{
	return myPosition.y;
}

float cObjectPath::getSpeedX()
{
	return mySpeed.x;
}

float cObjectPath::getSpeedY()
{
	return mySpeed.y;
}

float cObjectPath::calcX( float forY )
{
	// x= (y-intercept)/slope
	return ( ( forY - intercept )/slope );
}

float cObjectPath::calcY( float forX )
{
    // y= slope*x + intercept;
	return ( slope * forX + intercept );
}

float cObjectPath::getYIntercept()
{
	return intercept;
}

float cObjectPath::getSlope()
{
	return slope;
}

/*! return the extrapolated coordinate (1 meter) outside the field where the object is moving to
 *
 * @return coordinate
 */
Point2D cObjectPath::getTowardsCoordinate()
{
	float objectTowardsX, objectTowardsY;

    auto fieldWidth = fieldDimensionsStore::getFieldDimensions().getWidth();
    auto fieldLength = fieldDimensionsStore::getFieldDimensions().getLength();

    if( mySpeed.x < 0.0 )
    {
    	objectTowardsX = -fieldWidth/2.0 - 1.0;
    	objectTowardsY = calcY( objectTowardsX );
    }
    else if ( mySpeed.x > 0.0 )
    {
    	objectTowardsX =  fieldWidth/2.0 + 1.0;
    	objectTowardsY = calcY( objectTowardsX );
    }
    else
    {
    	objectTowardsX = myPosition.x;
    	if ( mySpeed.y > 0.0 )
    	{
    		objectTowardsY = fieldLength/2.0 + 1.0;
    	}
    	else if ( mySpeed.y < 0.0)
    	{
    		objectTowardsY = -fieldLength/2.0 - 1.0;
    	}
    	else
    	{
    		objectTowardsY = myPosition.y;
    	}
    }

    Point2D objectMovingTowardsCoordinate( objectTowardsX, objectTowardsY);

    return objectMovingTowardsCoordinate;

}

/*! check if the line segment x1,y1 to x2,y2 intersects with the line which is followed by the object
 *
 * @param[in] lineStart coord of line segment start
 * @param[in] lineEnd coord of line segment end
 * @return true if the two linesegments intersect
 */
bool cObjectPath::intersectCheck( Point2D lineStart, Point2D lineEnd)
{
	bool b;

    Point2D oMTP = getTowardsCoordinate();

    //translate to boost variable types Point and Segment
    Point objectMovingTowardsPosition( oMTP.x, oMTP.y );

    Segment objectLine( Point( myPosition.x, myPosition.y), objectMovingTowardsPosition);
    Segment line( Point(lineStart.x,lineStart.y), Point(lineEnd.x,lineEnd.y));

	b = boost::geometry::intersects(objectLine, line);
	return b;

}

/*! check if the line segment x1,y1 to x2,y2 intersects with the line which is followed by the object
 *
 * @param[in] lineStart coord of line segment start
 * @param[in] lineEnd coord of line segment end
 * @param[out] intersect coord of intersection point, only valid when function returns true!
 * @return true if the two linesegments intersect
 */
bool cObjectPath::intersectCheck( Point2D lineStart, Point2D lineEnd, Point2D &intersectCoord)
{
    Point2D oMTP = getTowardsCoordinate();
    //translate to boost variable types Point and Segment
    Point objectMovingTowardsPosition( oMTP.x, oMTP.y );

    Segment objectLine( Point( myPosition.x, myPosition.y), objectMovingTowardsPosition);
    Segment line( Point(lineStart.x,lineStart.y), Point(lineEnd.x,lineEnd.y));

	std::vector<Point> output;
	boost::geometry::intersection(objectLine, line, output);

	if ( output.size() == 1 )
	{
		intersectCoord.x=boost::geometry::get<0>( output[0] ); // get first field representing x of intersect
		intersectCoord.y=boost::geometry::get<1>( output[0] ); // get second field representing y of intersect
		return true;
	}
	return false;

}

/*! return the point on objectLine which is perpendicular to the position of otherObject
 *
 * @param[in] otherObject coord of the other object which wants to have a coordinate for interception
 * @param[out] valid true if the coordinate is on the line in the moving direction of the object
 * @return coordinate target on line
 */
Point2D cObjectPath::getPerpendicularInterceptionPoint( Point2D otherObject,  bool &valid )
{
    Point2D objectMovingTowardsPosition= getTowardsCoordinate();

    // formula found on http://arstechnica.com/civis/viewtopic.php?f=26&t=149128
    float u=( (otherObject.x - myPosition.x ) * ( objectMovingTowardsPosition.x - myPosition.x) +
		      (otherObject.y - myPosition.y ) * ( objectMovingTowardsPosition.y - myPosition.y) );
	float dist=calc_distance( Point2D( myPosition.x,myPosition.y), objectMovingTowardsPosition);

	u= u/(dist*dist);

	Point2D target(0.0, 0.0);

	target.x=myPosition.x + u * ( objectMovingTowardsPosition.x - myPosition.x);
	target.y=myPosition.y + u * ( objectMovingTowardsPosition.y - myPosition.y);

    valid=true;

	if ( mySpeed.x > 0.0 )
	{
		if ( target.x < myPosition.x )
			valid=false;
	}
	else if ( mySpeed.x < 0.0)
	{
		if ( target.x > myPosition.x )
			valid=false;
	}

	if( valid )
	{
		if ( mySpeed.y > 0.0)
		{
			if ( target.y < myPosition.y )
				valid=false;
		}
		else if ( mySpeed.y < 0.0)
		{
			if ( target.y > myPosition.y)
				valid=false;
		}
	}

    return target;
}


