// Copyright 2016 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotLocation.hpp
 *
 *  Created on: Oct 9, 2016
 *      Author: robocup
 */

#ifndef ROBOTLOCATION_HPP_
#define ROBOTLOCATION_HPP_

typedef struct
{
	float x;
	float y;
	float theta;
	float vx;
	float vy;
	float vtheta;
	float confidence;
	double timestamp;
} robotLocationStructure;

#endif /* ROBOTLOCATION_HPP_ */
