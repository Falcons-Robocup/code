// Copyright 2016 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef CAMFEEDPARAMS_H
#define CAMFEEDPARAMS_H

#include <string>

struct CamFeedParams
{
    // camera position w.r.t. field center
    float camPositionX = -8.224;
    float camPositionY = -8.441;
    float camPositionZ = 2.888;
    // scaling distance to put the image behind the scene
    float backgroundDistance = 2836.467;
    // correction offset
    float camOffsetX = 15.261;
    float camOffsetY = 173.157;
    float camOffsetZ = -4.213;
    // rotation offset to align up-vectors
    float upCorrection = 6.575;
    // camera zoom scaling (non-parallel) 
    float viewAngle = 40.048;

    // load from file
    void load(std::string filename);
    
};

#endif // CAMFEEDPARAMS_H

