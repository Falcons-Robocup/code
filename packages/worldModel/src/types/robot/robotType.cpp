// Copyright 2016-2020 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotType.cpp
 *
 *  Created on: Aug 16, 2016
 *      Author: Tim Kouters
 */

#include "int/types/robot/robotType.hpp"

#include "falconsCommon.hpp"

robotClass_t::robotClass_t()
{
    _robotID = 0;
    _timestamp = 0.0;
    _coordinates = coordinateType::FIELD_COORDS;
    _x = 0.0;
    _y = 0.0;
    _theta = 0.0;
    _vx = 0.0;
    _vy = 0.0;
    _vtheta = 0.0;
    _ballPossession = false;
}

robotClass_t::~robotClass_t()
/*
 * Chuck Norris doesn't dial the wrong number, you pick up the wrong phone
 */
{

}

void robotClass_t::setRobotID(const uint8_t robotID)
{
    _robotID = robotID;
}

void robotClass_t::setTimestamp(const double stamp)
{
    _timestamp = stamp;
}

void robotClass_t::setCoordinateType(const coordinateType coordinate)
{
    _coordinates = coordinate;
}

void robotClass_t::setCoordinates(const float x, const float y, const float theta)
{
    _x = x;
    _y = y;
    _theta = theta;
}

void robotClass_t::setVelocities(const float vx, const float vy, const float vtheta)
{
    _vx = vx;
    _vy = vy;
    _vtheta = vtheta;
}

void robotClass_t::setBallPossession(bool bp)
{
    _ballPossession = bp;
}

uint8_t robotClass_t::getRobotID() const
{
    return _robotID;
}

double robotClass_t::getTimestamp() const
{
    return _timestamp;
}

coordinateType robotClass_t::getCoordindateType() const
{
    return _coordinates;
}


float robotClass_t::getX() const
{
    return _x;
}

float robotClass_t::getY() const
{
    return _y;
}

float robotClass_t::getTheta() const
{
    return project_angle_0_2pi(_theta);
}

float robotClass_t::getVX() const
{
    return _vx;
}

float robotClass_t::getVY() const
{
    return _vy;
}

float robotClass_t::getVTheta() const
{
    return _vtheta;
}

bool robotClass_t::getBallPossession() const
{
    return _ballPossession;
}

