// Copyright 2018 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotVelocityType.cpp
 *
 *  Created on: Nov 8, 2018
 *      Author: Erik Kouters
 */

#include "int/types/robot/robotVelocityType.hpp"

robotVelocityClass_t::robotVelocityClass_t()
{
	_coordinate = coordinateType::FIELD_COORDS;
	_displacementSource = displacementType::INVALID;
	_timestamp = 0.0;
	_vx = 0.0;
	_vy = 0.0;
	_vtheta = 0.0;
}

robotVelocityClass_t::~robotVelocityClass_t()
/*
 * Chuck Norris can cut a knife with butter
 */
{

}

void robotVelocityClass_t::setID(const uniqueObjectID identifier)
{
	_identifier = identifier;
}

void robotVelocityClass_t::setCoordinateType(const coordinateType coordinates)
{
	_coordinate = coordinates;
}

void robotVelocityClass_t::setDisplacementSource(const displacementType displacementSource)
{
	_displacementSource = displacementSource;
}

void robotVelocityClass_t::setTimestamp(const double timestamp)
{
	_timestamp = timestamp;
}

void robotVelocityClass_t::setDeltaVelocity(const float vx, const float vy, const float vtheta)
{
	_vx = vx;
	_vy = vy;
	_vtheta = vtheta;
}


uniqueObjectID robotVelocityClass_t::getID() const
{
	return _identifier;
}

coordinateType robotVelocityClass_t::getCoordinateType() const
{
	return _coordinate;
}

displacementType robotVelocityClass_t::getDisplacementSource() const
{
	return _displacementSource;
}

double robotVelocityClass_t::getTimestamp() const
{
	return _timestamp;
}

float robotVelocityClass_t::getvX() const
{
	return _vx;
}

float robotVelocityClass_t::getvY() const
{
	return _vy;
}

float robotVelocityClass_t::getvTheta() const
{
	return _vtheta;
}

