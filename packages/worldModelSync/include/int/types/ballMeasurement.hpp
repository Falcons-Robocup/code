 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
