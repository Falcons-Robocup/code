 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ball.cpp
 *
 *  Created on: Nov 21, 2018
 *      Author: Coen Tempelaars
 */

#include "int/ball.hpp"

#include "tracing.hpp"

const static float minimalMovingSpeed = 0.01;

Point2D Ball::getLocation() const
{
    return Point2D(_position.x, _position.y);
}

Point2D Ball::getPickupLocation() const
{
    return _pickupLocation;
}

Point3D Ball::getPosition() const
{
    return _position;
}

Point3D Ball::getPositionFCS(const TeamID& teamID) const
{
    if (teamID == TeamID::A)
    {
        return _position;
    }
    else
    {
        return (_position * -1.0);
    }
}

float Ball::getSpeed() const
{
    return vectorsize(_velocity);
}

Vector3D Ball::getVelocity() const
{
    return _velocity;
}

bool Ball::isMoving() const
{
    if (vectorsize(_velocity) > minimalMovingSpeed)
    {
        TRACE("The ball is moving");
        return true;
    }
    else
    {
        TRACE("The ball is not moving");
        return false;
    }
}

void Ball::setLocation (const Point2D& p)
{
    this->setPosition(Point3D(p.x, p.y, 0.0));
}

void Ball::setPickupLocation (const Point2D& p)
{
    _pickupLocation = p;
}

void Ball::setPosition (const Point3D& p)
{
    _position = p;
}

void Ball::setVelocity (const Vector3D& v)
{
    _velocity = v;
}

void Ball::stopMoving()
{
    _velocity = Vector3D();
}

void Ball::teleport(const Vector2D& v)
{
    this->teleport(v.x, v.y);
}

void Ball::teleport (const float x, const float y)
{
    this->teleport(x, y, 0.0);
}

void Ball::teleport (const float x, const float y, const float z)
{
    _position = Point3D(x, y, z);
    _velocity = Vector3D();
}
