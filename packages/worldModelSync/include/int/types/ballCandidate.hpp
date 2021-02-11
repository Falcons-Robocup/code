// Copyright 2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ballCandidate.hpp
 *
 *  Created on: Jan 21, 2017
 *      Author: Tim Kouters
 */

#ifndef BALLCANDIDATE_HPP_
#define BALLCANDIDATE_HPP_

#include <stdint.h>

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t vx;
	int16_t vy;
	int16_t vz;
	uint8_t confidence;
} ballCandidateStructure;

#endif /* BALLCANDIDATE_HPP_ */
