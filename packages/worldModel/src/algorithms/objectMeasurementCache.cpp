 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * objectMeasurementCache.cpp
 *
 *  Created on: Oct 30, 2016
 *      Author: Jan Feitsma
 */

#include "int/algorithms/objectMeasurementCache.hpp"
#include "linalgcv.hpp" // from package geometry

objectMeasurementCache::objectMeasurementCache(objectMeasurement const &objectMeasurement)
{
    _objectMeasurement = objectMeasurement;
    // calculate FCS position
    _positionFcs = object2fcs(objectMeasurement.cameraX, 
                            objectMeasurement.cameraY, 
                            objectMeasurement.cameraZ, 
                            objectMeasurement.cameraPhi, 
                            objectMeasurement.azimuth, 
                            objectMeasurement.elevation, 
                            objectMeasurement.radius);
    // calculate and cache design matrix
    calculateCvMatrix();
}

objectMeasurementCache::~objectMeasurementCache()
{
}

Vector3D objectMeasurementCache::getPositionFcs() const
{
    return _positionFcs;
}

cv::Mat objectMeasurementCache::getCvMatrix() const
{
    return _matrix;
}

objectMeasurement objectMeasurementCache::getObjectMeasurement() const
{
    return _objectMeasurement;
}

void objectMeasurementCache::calculateCvMatrix()
{
    float cx = _objectMeasurement.cameraX;
    float cy = _objectMeasurement.cameraY;
    float cz = _objectMeasurement.cameraZ;
    float cphi = _objectMeasurement.cameraPhi;
    float az = _objectMeasurement.azimuth;
    float el = _objectMeasurement.elevation;
    _matrix = constructD(cx, cy, cz, cphi, az, el);
}

