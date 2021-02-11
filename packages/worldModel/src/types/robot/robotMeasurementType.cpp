// Copyright 2016-2018 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotMeasurementType.cpp
 *
 *  Created on: Aug 16, 2016
 *      Author: Tim Kouters
 */

#include "int/types/robot/robotMeasurementType.hpp"

robotMeasurementClass_t::robotMeasurementClass_t()
{
	_coordinate = coordinateType::ROBOT_COORDS;
	_timestamp = 0.0;
	_confidence = 0.0;
	_x = 0.0;
	_y = 0.0;
	_theta = 0.0;
}

robotMeasurementClass_t::~robotMeasurementClass_t()
/*
 * Chuck Norris can knit the softest sweater using electrical pylons
 */
{

}

void robotMeasurementClass_t::setID(const uniqueObjectID identifier)
{
	_identifier = identifier;
}

void robotMeasurementClass_t::setCoordinateType(const coordinateType coordinates)
{
	_coordinate = coordinates;
}

void robotMeasurementClass_t::setTimestamp(const double timestamp)
{
	_timestamp = timestamp;
}

void robotMeasurementClass_t::setConfidence(const float confidence)
{
	_confidence = confidence;
}

void robotMeasurementClass_t::setPosition(const float x, const float y, const float theta)
{
	_x = x;
	_y = y;
	_theta = theta;
}


uniqueObjectID robotMeasurementClass_t::getID() const
{
	return _identifier;
}

coordinateType robotMeasurementClass_t::getCoordindateType() const
{
	return _coordinate;
}

double robotMeasurementClass_t::getTimestamp() const
{
	return _timestamp;
}

float robotMeasurementClass_t::getConfidence() const
{
	return _confidence;
}

float robotMeasurementClass_t::getX() const
{
	return _x;
}

float robotMeasurementClass_t::getY() const
{
	return _y;
}

float robotMeasurementClass_t::getTheta() const
{
	return _theta;
}

Position2D robotMeasurementClass_t::getPosition() const
{
    return Position2D(_x, _y, _theta);
}

