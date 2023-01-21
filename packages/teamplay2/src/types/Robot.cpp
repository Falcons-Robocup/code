// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Robot.cpp
 *
 *  Created on: Nov 27, 2016
 *      Author: Coen Tempelaars
 */

#include "boost/format.hpp"

#include "falconsCommon.hpp"

#include "int/types/Robot.hpp"

using namespace teamplay;


Robot::Robot()
    : _number(0)
    , _role()
    , _position()
    , _velocity()
    , _hasBall(false)
    , _isOwnRobot(false)
{ }

Robot::Robot(const RobotNumber number)
: _number(number)
, _role()
, _position()
, _velocity()
, _hasBall(false)
, _isOwnRobot(false)
{ }

Robot::Robot(const RobotNumber number, const RoleEnum& r, const geometry::Pose2D& pos, const geometry::Velocity2D& vel)
    : _number(number)
    , _role(r)
    , _position(pos)
    , _velocity(vel)
    , _hasBall(false)
    , _isOwnRobot(false)
{ }

Robot::~Robot()
{ }

bool Robot::operator== (const Robot& other) const
{
    return ( _number == other._number );
}

bool Robot::operator!= (const Robot& other) const
{
    return !(*this == other);
}

bool Robot::operator< (const Robot& other) const
{
    return ( _number < other._number );
}

bool Robot::hasBall() const
{
    return _hasBall;
}

bool Robot::isInArea(const FieldArea& area) const
{
    return FieldDimensions().isPositionInArea(_position, area);
}

bool Robot::isOwnRobot() const
{
    return _isOwnRobot;
}

RobotNumber Robot::getNumber() const
{
    return _number;
}

RoleEnum Robot::getRole() const
{
    return _role.getRole();
}

boost::optional<RoleEnum> Robot::getAssistantRole() const
{
    return _role.getAssistantRole();
}

Point2D Robot::getLocation() const
{
    return Point2D(_position.x, _position.y);
}

geometry::Pose2D Robot::getPosition() const
{
    return _position;
}

geometry::Velocity2D Robot::getVelocity() const
{
    return _velocity;
}

double Robot::getDistanceTo (const Point2D& p) const
{
    return calc_distance(getLocation(), p);
}

void Robot::setNumber(const RobotNumber number)
{
    _number = number;
}

void Robot::setRole(const RoleEnum& t)
{
    _role = Role(t);
}

void Robot::setPosition(const geometry::Pose2D& position)
{
    _position = position;
}

void Robot::setVelocity(const geometry::Velocity2D& velocity)
{
    _velocity = velocity;
}

void Robot::claimsBallPossession()
{
    _hasBall = true;
}

void Robot::losesBallPossession()
{
    _hasBall = false;
}

void Robot::setOwnRobot()
{
    _isOwnRobot = true;
}

void Robot::setNotOwnRobot()
{
    _isOwnRobot = false;
}

std::string Robot::str() const
{
    boost::format fmt("Robot [%1%] [%2%] [%3%] [%4%]");
    fmt % std::to_string(_number);
    fmt % (_isOwnRobot ? "own" : "not own");
    fmt % (_hasBall ? "with Ball" : "without Ball");
    fmt % _role.str();
    return fmt.str();
}
