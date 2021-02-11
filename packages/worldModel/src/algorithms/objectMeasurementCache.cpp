// Copyright 2017-2018 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
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

