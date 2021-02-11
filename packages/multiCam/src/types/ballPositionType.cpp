// Copyright 2018-2019 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ballPositionType.cpp
 *
 *  Created on: Oct 25, 2016
 *      Author: Tim Kouters
 */

#include "types/ballPositionType.hpp"

ballPositionType::ballPositionType()
{
    _angle = 0.0;
    _radius = 0.0;
    _elevation = 0.0;
    _confidence = 0.0;
    _type = 0;
}

ballPositionType::~ballPositionType()
{

}


void ballPositionType::setAngle(const float angle)
{
    _angle = angle;
}

void ballPositionType::setRadius(const float radius)
{
    _radius = radius;
}

void ballPositionType::setElevation(const float elevation)
{
    _elevation = elevation;
}

void ballPositionType::setConfidence(const float confidence)
{
    _confidence = confidence;
}

void ballPositionType::setBallType(size_t type)
{
    _type = type;
}



float ballPositionType::getAngle() const
{
    return _angle;
}

float ballPositionType::getRadius() const
{
    return _radius;
}

float ballPositionType::getElevation() const
{
    return _elevation;
}

float ballPositionType::getConfidence() const
{
    return _confidence;
}

size_t ballPositionType::getBallType() const
{
    return _type;
}

