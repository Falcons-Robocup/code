// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * visionObjects.hpp
 *
 *  Created on: Nov 12, 2020
 *      Author: Erik Kouters
 *
 */

#ifndef VISIONOBJECTS_HPP_
#define VISIONOBJECTS_HPP_

#include "RtDB2.h" // required for serialization

struct visionObject {

    float azimuth;      // -M_PI/2..M_PI/2 (-90..90 degrees)
    int classId;        // 0=ball, 1=obstacle, 2=human, 3=goalPost and 4=border
    float confidence;   // 0..1 (range does not include 1.0 itself)
    float elevation;    // -M_PI/2..M_PI/2 (-90..90 degrees)
    float height;       // 0..1 (range does not include 1.0 itself)
    float radius;       // 0..MAX_FLT in meters
    float width;        // 0..1 (range does not include 1.0 itself)
    float xCenter;      // 0..1 (range does not include 1.0 itself)
    float yCenter;      // 0..1 (range does not include 1.0 itself)

    SERIALIZE_DATA(azimuth, classId, confidence, elevation, height, radius, width, xCenter, yCenter);
};


struct visionObjects
{

    int frame;
    int robot;
    int camera;
    uint64_t milliSeconds;
    std::vector<visionObject> objects;
    
    SERIALIZE_DATA(frame, robot, camera, milliSeconds, objects);
};

#endif

