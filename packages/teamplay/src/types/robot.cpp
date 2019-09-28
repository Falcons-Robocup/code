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
 *  Created on: Nov 27, 2016
 *      Author: Coen Tempelaars
 */

#include "boost/format.hpp"

#include "FalconsCommon.h"

#include "int/types/robot.hpp"

using namespace teamplay;


robot::robot()
    : _number(0)
    , _role()
    , _position()
    , _velocity()
    , _hasBall(false)
    , _isOwnRobot(false)
{ }

robot::robot(const robotNumber number)
: _number(number)
, _role()
, _position()
, _velocity()
, _hasBall(false)
, _isOwnRobot(false)
{ }

robot::robot(const robotNumber number, const treeEnum& t, const Position2D& pos, const Velocity2D& vel)
    : _number(number)
    , _role(t)
    , _position(pos)
    , _velocity(vel)
    , _hasBall(false)
    , _isOwnRobot(false)
{ }

robot::~robot()
{ }

bool robot::hasBall() const
{
    return _hasBall;
}

bool robot::isInArea(const fieldArea& area) const
{
    return fieldDimensions().isPositionInArea(_position, area);
}

bool robot::isOwnRobot() const
{
    return _isOwnRobot;
}

robotNumber robot::getNumber() const
{
    return _number;
}

treeEnum robot::getRole() const
{
    return _role.getRole();
}

boost::optional<treeEnum> robot::getAssistantRole() const
{
    return _role.getAssistantRole();
}

Point2D robot::getLocation() const
{
    return Point2D(_position.x, _position.y);
}

Position2D robot::getPosition() const
{
    return _position;
}

Velocity2D robot::getVelocity() const
{
    return _velocity;
}

double robot::getDistanceTo (const Point2D& p) const
{
    return calc_distance(getLocation(), p);
}

void robot::setNumber(const robotNumber number)
{
    _number = number;
}

void robot::setRole(const treeEnum& t)
{
    _role = role(t);
}

void robot::setPosition(const Position2D& position)
{
    _position = position;
}

void robot::setVelocity(const Velocity2D& velocity)
{
    _velocity = velocity;
}

void robot::claimsBallPossession()
{
    _hasBall = true;
}

void robot::losesBallPossession()
{
    _hasBall = false;
}

void robot::setOwnRobot()
{
    _isOwnRobot = true;
}

void robot::setNotOwnRobot()
{
    _isOwnRobot = false;
}

std::string robot::str() const
{
    boost::format fmt("Robot [%1%] [%2%] [%3%] [%4%]");
    fmt % std::to_string(_number);
    fmt % (_isOwnRobot ? "own" : "not own");
    fmt % (_hasBall ? "with ball" : "without ball");
    fmt % _role.str();
    return fmt.str();
}
