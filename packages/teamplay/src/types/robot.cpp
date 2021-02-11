// Copyright 2016-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robot.cpp
 *
 *  Created on: Nov 27, 2016
 *      Author: Coen Tempelaars
 */

#include "boost/format.hpp"

#include "falconsCommon.hpp"

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
