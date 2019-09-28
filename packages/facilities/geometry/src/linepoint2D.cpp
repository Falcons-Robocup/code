 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
