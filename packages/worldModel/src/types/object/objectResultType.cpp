// Copyright 2017 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * objectResultType.cpp
 *
 *  Created on: Jan 15, 2017
 *      Author: Jan Feitsma
 */

#include "int/types/object/objectResultType.hpp"

objectResultType::objectResultType()
{
	_timestamp = 0.0;
	_id = 0;
	_x = 0.0;
	_y = 0.0;
	_z = 0.0;
	_vx = 0.0;
	_vy = 0.0;
	_vz = 0.0;
}

objectResultType::~objectResultType()
/*
 * Chuck Norris threw a grenade and killed 50 people, then it exploded
 */
{

}

void objectResultType::setId(const size_t id)
{
	_id = id;
}

void objectResultType::setTimestamp(const double stamp)
{
	_timestamp = stamp;
}

void objectResultType::setCoordinates(const float x, const float y, const float z)
{
	_x = x;
	_y = y;
	_z = z;
}

void objectResultType::setVelocities(const float vx, const float vy, const float vz)
{
	_vx = vx;
	_vy = vy;
	_vz = vz;
}

size_t objectResultType::getId() const
{
	return _id;
}

double objectResultType::getTimestamp() const
{
	return _timestamp;
}

float objectResultType::getX() const
{
	return _x;
}

float objectResultType::getY() const
{
	return _y;
}

float objectResultType::getZ() const
{
	return _z;
}

float objectResultType::getVX() const
{
	return _vx;
}

float objectResultType::getVY() const
{
	return _vy;
}

float objectResultType::getVZ() const
{
	return _vz;
}


