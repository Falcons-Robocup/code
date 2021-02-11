// Copyright 2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * obstacleCandidate.hpp
 *
 *  Created on: Jan 21, 2017
 *      Author: Tim Kouters
 */

#ifndef OBSTACLECANDIDATE_HPP_
#define OBSTACLECANDIDATE_HPP_

#include <stdint.h>

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t vx;
	int16_t vy;
	uint8_t  confidence;
} obstacleCandidateStructure;

#endif /* OBSTACLECANDIDATE_HPP_ */
