// Copyright 2016 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * obstacleMeasurement.hpp
 *
 *  Created on: Oct 9, 2016
 *      Author: Tim Kouters
 */

#ifndef OBSTACLEMEASUREMENT_HPP_
#define OBSTACLEMEASUREMENT_HPP_

#include <stddef.h>
#include <stdint.h>

typedef struct
{
	bool isValid;
	size_t objectID;
	float azimuth;
	float elevation;
	float radius;
	float cameraX;
	float cameraY;
	float cameraZ;
	float cameraPhi;
	float confidence;
	double timestamp;
	uint8_t cameraType;
} obstacleMeasurement;

/*
 * Function needed for std::sort to sort vectors
 */
static bool sortObstacleMeasurementsOnTimeStamp(const obstacleMeasurement objA, const obstacleMeasurement objB)
{
	return objA.timestamp > objB.timestamp;
}

#endif /* OBSTACLEMEASUREMENT_HPP_ */
