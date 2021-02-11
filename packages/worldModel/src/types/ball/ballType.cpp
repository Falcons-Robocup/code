// Copyright 2016-2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ballType.cpp
 *
 *  Created on: Aug 16, 2016
 *      Author: robocup
 */

#include "int/types/ball/ballType.hpp"

ballClass_t::ballClass_t()
{
	_confidence = 0.0;
	_isValid = false;
}

ballClass_t::~ballClass_t()
/*
 * Chuck Norris threw a grenade and killed 50 people, then it exploded
 */
{

}

void ballClass_t::setConfidence(const float confidence)
{
	_confidence = confidence;
}

void ballClass_t::setIsValid(const bool isValid)
{
	_isValid = isValid;
}

float ballClass_t::getConfidence() const
{
	return _confidence;
}

bool ballClass_t::getIsValid() const
{
	return _isValid;
}

