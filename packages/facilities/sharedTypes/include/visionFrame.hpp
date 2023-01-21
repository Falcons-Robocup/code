// Copyright 2020-2022 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * visionObjects.hpp
 *
 *  Created on: Nov 12, 2020
 *      Author: Erik Kouters
 *
 */

#ifndef VISIONFRAME_HPP_
#define VISIONFRAME_HPP_

#include "RtDB2.h" // required for serialization
#include "RtDB2Definitions.h"

struct visionObject {
    float azimuth;         // -M_PI/2..M_PI/2 (-90..90 degrees)
    std::string className; // ball, obstacle, human, goalPost border // TODO enum? but then rtop is not readible ...
    float confidence;      // 0..1 (range does not include 1.0 itself)
    float elevation;       // -M_PI/2..M_PI/2 (-90..90 degrees)
    float height;          // 0..1 (range does not include 1.0 itself)
    float radius;          // 0..MAX_FLT in meters
    float width;           // 0..1 (range does not include 1.0 itself)
    float xCenter;         // 0..1 (range does not include 1.0 itself)
    float yCenter;         // 0..1 (range does not include 1.0 itself)

    SERIALIZE_DATA(azimuth, className, confidence, elevation, height, radius, width, xCenter, yCenter);
};

struct visionFrame {
    int frame;             // counter
    int robot;
    rtime timestamp;       // timestamp of camera shutter, for ultimo tracking sync
    std::vector<visionObject> objects;

    SERIALIZE_DATA(frame, robot, timestamp, objects);
};

#endif
