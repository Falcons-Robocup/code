// Copyright 2018-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
