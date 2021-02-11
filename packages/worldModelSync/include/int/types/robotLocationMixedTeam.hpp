// Copyright 2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotLocationMixedTeam.hpp
 *
 *  Created on: Jan 21, 2017
 *      Author: Tim Kouters
 */

#ifndef ROBOTLOCATIONMIXEDTEAM_HPP_
#define ROBOTLOCATIONMIXEDTEAM_HPP_

#include <stdint.h>

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t theta;
	int16_t vx;
	int16_t vy;
	int16_t vtheta;
	uint8_t confidence;
} robotLocationMixedTeamStructure;

#endif /* ROBOTLOCATIONMIXEDTEAM_HPP_ */
