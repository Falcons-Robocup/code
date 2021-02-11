// Copyright 2016-2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * obstacleType.cpp
 *
 *  Created on: Aug 16, 2016
 *      Author: Tim Kouters
 */

#include "int/types/obstacle/obstacleType.hpp"

obstacleClass_t::obstacleClass_t()
{
	_confidence = 0.0;
}

obstacleClass_t::~obstacleClass_t()
/*
 * Chuck Norris can speak French... In Russian.
 */
{

}

void obstacleClass_t::setConfidence(const float confidence)
{
	_confidence = confidence;
}

float obstacleClass_t::getConfidence() const
{
	return _confidence;
}

