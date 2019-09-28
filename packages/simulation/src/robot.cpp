 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * robot.cpp
 *
 *  Created on: Nov 21, 2018
 *      Author: Coen Tempelaars
 */

#include "int/robot.hpp"
#include "int/ballCapabilities.hpp"
#include "int/robotCapabilities.hpp"

#include "tracing.hpp"
#include <cmath>
#include <iostream>

const static float BALL_PICKUP_THRESHOLD = (BALL_RADIUS + ROBOT_RADIUS) * 1.05;  // 5% tolerance
const static float BALL_KICK_THRESHOLD = 0.1;

Circle Robot::getCircumference() const
{
    return Circle(_position.x, _position.y, ROBOT_RADIUS);
}

float Robot::getDistanceTo (const Point2D& p) const
{
    Vector2D posdelta = (this->getLocation() - p);
    return posdelta.size();
}

Point2D Robot::getLocation() const
{
    return Point2D(_position.x, _position.y);
}

Point2D Robot::getMouthLocation(const float offset) const
{
    return Point2D(
            _position.x + cos(_position.phi) * offset,
            _position.y + sin(_position.phi) * offset);
}

Position2D Robot::getPosition() const
{
    return _position;
}

Position2D Robot::getPositionFCS() const
{
    auto position_fcs = _position;
    position_fcs.transform_acs2fcs(_playingDirection == PlayingDirection::LEFT_TO_RIGHT);
    return position_fcs;
}

float Robot::getAngle() const
{
    return _position.phi;
}

Vector2D Robot::getVelocityVector() const
{
    return Vector2D(_velocity.x, _velocity.y);
}

Velocity2D Robot::getVelocity() const
{
    return _velocity;
}

Velocity2D Robot::getVelocityFCS() const
{
    auto velocity_fcs = _velocity;
    velocity_fcs.transform_acs2fcs(_playingDirection == PlayingDirection::LEFT_TO_RIGHT);
    return velocity_fcs;
}

bool Robot::isMoving() const
{
    if (_velocity.size() > ROBOT_MINIMAL_MOVING_SPEED)
    {
        TRACE("This robot is moving");
        return true;
    }
    else
    {
        TRACE("This robot is not moving");
        return false;
    }
}

bool Robot::isKicking() const
{
    if (_kicker.speed > ROBOT_MINIMAL_KICKING_SPEED)
    {
        TRACE("This robot is kicking");
        return true;
    }
    else
    {
        TRACE("This robot is not kicking");
        return false;
    }
}

bool Robot::canGrabBall (const Point3D& ballPosition) const
{
    bool retval = false;

    Vector2D ballPositionVec2D(ballPosition.x, ballPosition.y);
    Vector2D robotPositionVec2D(_position.x, _position.y);
    auto robotBallDistance = (ballPositionVec2D - robotPositionVec2D).size();

    if (robotBallDistance < BALL_PICKUP_THRESHOLD)
    {
        // Calculate the position of the robot's "ball mouth"
        auto mouthPosition = this->getMouthLocation(ROBOT_RADIUS);
        auto mouthBallDistance = (ballPositionVec2D - mouthPosition).size();

        if ((mouthBallDistance < ROBOT_RADIUS) && hasBallHandlersEnabled())
        {
            retval = true;
        }
    }

    if (retval)
    {
        TRACE("This robot can grab the ball");
    }
    else
    {
        TRACE("This robot cannot grab the ball");
    }

    return retval;
}

bool Robot::canKickBall (const Point3D& ballPosition) const
{
    bool retval = false;

    Vector2D ballPositionVec2D(ballPosition.x, ballPosition.y);
    auto mouthPosition = this->getMouthLocation(ROBOT_RADIUS);
    auto mouthBallDistance = (ballPositionVec2D - mouthPosition).size();

    if (_ballHandlingModule == BallHandlingModule::PRESENT)
    {
        if (mouthBallDistance < BALL_KICK_THRESHOLD)
        {
            retval = true;
        }
    }

    if (retval)
    {
        TRACE("This robot can kick the ball");
    }
    else
    {
        TRACE("This robot cannot kick the ball");
    }

    return retval;
}

bool Robot::hasBallHandlersEnabled() const
{
    if (_ballHandlingModule == BallHandlingModule::PRESENT)
    {
        if (_ballHandlersEnabled)
        {
            TRACE("Ball handlers are enabled");
            return true;
        }
        else
        {
            TRACE("Ball handlers are disabled");
            return false;
        }
    }
    else
    {
        TRACE("Ball handlers are not present");
        return false;
    }
}

float Robot::getKickerHeight() const
{
    return _kicker.height;
}

float Robot::getKickerSpeed() const
{
    return _kicker.speed;
}

void Robot::recalculatePosition (const float dt)
{
    _position.transform_acs2fcs(_playingDirection == PlayingDirection::LEFT_TO_RIGHT);
    _position.update(getVelocityFCS(), dt);
    _position.transform_fcs2acs(_playingDirection == PlayingDirection::LEFT_TO_RIGHT);
}

void Robot::setBallHandlingModuleAbsent()
{
    _ballHandlingModule = BallHandlingModule::ABSENT;
    _ballHandlersEnabled = false;
}

void Robot::setBallHandlingModulePresent()
{
    _ballHandlingModule = BallHandlingModule::PRESENT;
}

void Robot::setPlayingDirection(const PlayingDirection& p)
{
    _playingDirection = p;
}

void Robot::setPosition (const Position2D& p)
{
    _position = p;
}

void Robot::setPositionFCS (const Position2D& p)
{
    _position = p;
    _position.transform_fcs2acs(_playingDirection == PlayingDirection::LEFT_TO_RIGHT);

}

void Robot::setVelocity (const Velocity2D& v)
{
    _velocity = v;
}

void Robot::setVelocityRCS (const Velocity2D& v)
{
    auto position_fcs = _position;
    position_fcs.transform_acs2fcs(_playingDirection == PlayingDirection::LEFT_TO_RIGHT);

    _velocity = v;
    _velocity.transform_rcs2fcs(position_fcs);
    _velocity.transform_fcs2acs(_playingDirection == PlayingDirection::LEFT_TO_RIGHT);
}

void Robot::enableBallHandlers()
{
    if (_ballHandlingModule == BallHandlingModule::PRESENT)
    {
        _ballHandlersEnabled = true;
    }
}

void Robot::disableBallHandlers()
{
    _ballHandlersEnabled = false;
}

void Robot::setKickerHeight (const float h)
{
    if (_ballHandlingModule == BallHandlingModule::PRESENT)
    {
        _kicker.height = h;
    }
}

void Robot::setKickerSpeed (const float speed, const float scale)
{
    if (_ballHandlingModule == BallHandlingModule::PRESENT)
    {
        _kicker.speed = speed * scale;
    }
}

void Robot::trace() const
{
    std::cout << "absolute position: " << _position.to_string() << std::endl;
    std::cout << "absolute velocity: " << _velocity.to_string() << std::endl;
}