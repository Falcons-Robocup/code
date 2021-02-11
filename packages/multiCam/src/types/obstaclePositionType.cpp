// Copyright 2018 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * obstaclePositionType.cpp
 *
 *  Created on: Oct 25, 2016
 *      Author: Tim Kouters
 */

#include "types/obstaclePositionType.hpp"

obstaclePositionType::obstaclePositionType()
{
	_angle = 0.0;
	_radius = 0.0;
	_confidence = 0.0;
	_color = 0;
}

obstaclePositionType::~obstaclePositionType()
{

}

void obstaclePositionType::setAngle(const float angle)
{
	_angle = angle;
}

void obstaclePositionType::setRadius(const float radius)
{
	_radius = radius;
}

void obstaclePositionType::setConfidence(const float confidence)
{
	_confidence = confidence;
}

void obstaclePositionType::setColor(const int color)
{
	_color = color;
}

float obstaclePositionType::getAngle() const
{
	return _angle;
}

float obstaclePositionType::getRadius() const
{
	return _radius;
}

float obstaclePositionType::getConfidence() const
{
	return _confidence;
}

int obstaclePositionType::getColor() const
{
	return _color;
}
