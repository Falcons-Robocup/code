 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * objectMeasurement.hpp
 *
 *  Created on: Jan 10, 2017
 *      Author: Jan Feitsma
 *
 * Capturing the commonality of ball- and obstacle measurements.
 */

#ifndef OBJECTMEASUREMENT_HPP_
#define OBJECTMEASUREMENT_HPP_

#include "uniqueObjectID.hpp"
#include "cameraEnum.hpp"
#include "objectColorEnum.hpp"

#include "RtDB2.h" // required for serialization


struct objectMeasurement
{
    uniqueObjectID     identifier;
    rtime              timestamp; // instead of timeval, for performance and ease of computation
    cameraEnum         source;
    float              confidence;
    float              azimuth;
    float              elevation;
    float              radius;
    float              cameraX; // camera position in FCS is not known to vision, but added by worldModel before syncing
    float              cameraY;
    float              cameraZ;
    float              cameraPhi;
    objectColorEnum    color; // optionally filled in by vision
    
    bool operator==(const objectMeasurement &o) const;
    bool operator<(const objectMeasurement &o) const;
    
    SERIALIZE_DATA_FIXED(identifier, timestamp, source, confidence, azimuth, elevation, radius, cameraX, cameraY, cameraZ, cameraPhi, color);
};

#endif
