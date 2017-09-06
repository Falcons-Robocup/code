 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cTeamplayCommon.cpp
 *
 *  Created on: Sep 11, 2014
 *      Author: Jan Feitsma
 */

#include "int/cTeamplayCommon.hpp"

#include <math.h>

// Source: http://math.stackexchange.com/questions/1201337/finding-the-angle-between-two-points
double angle_between_two_points_0_2pi(double x1, double y1, double x2, double y2)
{

    double angle = atan2(y2 - y1, x2 - x1);
    return project_angle_0_2pi(angle);
}

// position type transformations
Position2D getPosition2D( const geometry::Pose2D& pose)
{
	Position2D myPosition;
	myPosition.x= pose.x;
	myPosition.y= pose.y;
	return ( myPosition );
}

Position2D getPosition2D( const Point3D& point3d)
{
	Position2D myPosition;
	myPosition.x= (double) point3d.x;
	myPosition.y= (double) point3d.y;
	return ( myPosition );
}

double calc_hypothenusa(double dX, double dY)
{
    // Function to calculate the Hypothenusa (Schuine zijde)
    double S = 0.0;
    S = sqrt(pow(dX, 2) + pow(dY, 2));
    return S;
}

float calc_hypothenusa(float dX, float dY)
{
    // Function to calculate the Hypothenusa (Schuine zijde)
    float S = 0.0;
    S = sqrt(pow(dX, 2) + pow(dY, 2));
    return S;
}

float calc_distance( float x1, float y1, float x2, float y2)
{
	float dX=x2-x1;
	float dY=y2-y1;

	return calc_hypothenusa( dX, dY );
}

double calc_distance( Position2D p1, Position2D p2 )
{
	double dX=p2.x - p1.x;
	double dY=p2.y - p1.y;

	return calc_hypothenusa( dX, dY );
}

double calc_distance( Point2D p1, Point2D p2 )
{
	double dX=p2.x - p1.x;
	double dY=p2.y - p1.y;

	return calc_hypothenusa( dX, dY );
}

// based on http://stackoverflow.com/a/385355
bool intersect(Vector2D const &a1, Vector2D const &a2, Vector2D const &b1, Vector2D const &b2, Vector2D &result)
{
    float x12 = a1.x - a2.x;
    float x34 = b1.x - b2.x;
    float y12 = a1.y - a2.y;
    float y34 = b1.y - b2.y;

    float c = x12 * y34 - y12 * x34;

    if (fabs(c) < 0.01)
    {
      // No intersection
      return false;
    }
      // Intersection
      float a = a1.x * a2.y - a1.y * a2.x;
      float b = b1.x * b2.y - b1.y * b2.x;

      result.x = (a * x34 - b * x12) / c;
      result.y = (a * y34 - b * y12) / c;

      return true;
}
