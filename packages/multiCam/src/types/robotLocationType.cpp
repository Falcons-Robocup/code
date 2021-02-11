// Copyright 2018 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotLocationType.cpp
 *
 *  Created on: Oct 25, 2016
 *      Author: Tim Kouters
 */

#include "types/robotLocationType.hpp"

robotLocationType::robotLocationType()
{
	_x = 0.0;
	_y = 0.0;
	_theta = 0.0;
	_confidence = 0.0;
	_fps = 0.0;
	_linePoints = 0;
	_age = 0.0;
	_lastActive = 0.0;
}

robotLocationType::~robotLocationType()
{

}

void robotLocationType::setX(const float x)
{
	_x = x;
}

void robotLocationType::setY(const float y)
{
	_y = y;
}

void robotLocationType::setTheta(const float theta)
{
	_theta = theta;
}

void robotLocationType::setConfidence(const float confidence)
{
	_confidence = confidence;
}

void robotLocationType::setFPS(const float fps)
{
	_fps = fps;
}

void robotLocationType::setLinePoints(const int linePoints)
{
	_linePoints = linePoints;
}

void robotLocationType::setAge(const float age)
{
	_age = age;
}

void robotLocationType::setLastActive(const float lastActive)
{
	_lastActive = lastActive;
}

float robotLocationType::getX() const
{
	return _x;
}

float robotLocationType::getY() const
{
	return _y;
}

float robotLocationType::getTheta() const
{
	return _theta;
}

float robotLocationType::getConfidence() const
{
	return _confidence;
}

float robotLocationType::getFPS() const
{
	return _fps;
}

int robotLocationType::getLinePoints() const
{
	return _linePoints;
}

float robotLocationType::getAge() const
{
	return _age;
}

float robotLocationType::lastActive() const
{
	return _lastActive;
}
