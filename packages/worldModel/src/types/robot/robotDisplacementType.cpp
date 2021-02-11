// Copyright 2016-2018 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotDisplacementType.cpp
 *
 *  Created on: Aug 16, 2016
 *      Author: Tim Kouters
 */

#include "int/types/robot/robotDisplacementType.hpp"

robotDisplacementClass_t::robotDisplacementClass_t()
{
	_coordinate = coordinateType::FIELD_COORDS;
	_displacementSource = displacementType::INVALID;
	_timestamp = 0.0;
	_dx = 0.0;
	_dy = 0.0;
	_dtheta = 0.0;
}

robotDisplacementClass_t::~robotDisplacementClass_t()
/*
 * Chuck Norris can cut a knife with butter
 */
{

}

void robotDisplacementClass_t::setID(const uniqueObjectID identifier)
{
	_identifier = identifier;
}

void robotDisplacementClass_t::setCoordinateType(const coordinateType coordinates)
{
	_coordinate = coordinates;
}

void robotDisplacementClass_t::setDisplacementSource(const displacementType displacementSource)
{
	_displacementSource = displacementSource;
}

void robotDisplacementClass_t::setTimestamp(const double timestamp)
{
	_timestamp = timestamp;
}

void robotDisplacementClass_t::setDeltaPosition(const float dx, const float dy, const float dtheta)
{
	_dx = dx;
	_dy = dy;
	_dtheta = dtheta;
}


uniqueObjectID robotDisplacementClass_t::getID() const
{
	return _identifier;
}

coordinateType robotDisplacementClass_t::getCoordinateType() const
{
	return _coordinate;
}

displacementType robotDisplacementClass_t::getDisplacementSource() const
{
	return _displacementSource;
}

double robotDisplacementClass_t::getTimestamp() const
{
	return _timestamp;
}

float robotDisplacementClass_t::getdX() const
{
	return _dx;
}

float robotDisplacementClass_t::getdY() const
{
	return _dy;
}

float robotDisplacementClass_t::getdTheta() const
{
	return _dtheta;
}

