// Copyright 2016-2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ballMeasurement.hpp
 *
 *  Created on: Oct 9, 2016
 *      Author: Tim Kouters
 */

#ifndef BALLMEASUREMENT_HPP_
#define BALLMEASUREMENT_HPP_

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
} ballMeasurement;

/*
 * Function needed for std::sort to sort vectors
 */
static bool sortBallMeasurementsOnCameraTypeAndTimeStamp(const ballMeasurement objA, const ballMeasurement objB)
{
    // return true means A comes in front.
    // We want omni vision in front, so check on camera type first.
    // If camera is equal, order on timestamp (newest on front).

    // cameraType 0 == OMNI, cameraType 1 == FRONT_VISION.
    if (objA.cameraType < objB.cameraType)
    {
        return true;
    }
    else if (objA.cameraType > objB.cameraType)
    {
        return false;
    }
    else // objA.cameraType == objB.cameraType
    {
        return objA.timestamp > objB.timestamp;
    }
}

#endif /* BALLMEASUREMENT_HPP_ */
