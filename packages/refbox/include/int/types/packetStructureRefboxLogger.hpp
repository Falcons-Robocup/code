// Copyright 2016-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * packetStructureRefboxLogger.hpp
 *
 *  Created on: Nov 24, 2015
 *      Author: Tim Kouters
 */

#ifndef PACKETSTRUCTUREREFBOXLOGGER_HPP_
#define PACKETSTRUCTUREREFBOXLOGGER_HPP_

#include <cstdio>
#include <stdint.h>
#include <vector>

// sharedTypes
#include "pose.hpp"
#include "vec2d.hpp"
#include "vec3d.hpp"

namespace packetRefboxLogger
{
    typedef struct
    {
        uint8_t     robotId;
        pose        position;
        pose        velocity;
        pose        targetPose;
        std::string intention;
        float       batteryLevel;
        bool        hasBall;
    } robotStructure;
    typedef std::vector<robotStructure> robotList;

    typedef struct
    {
        vec3d    position;
        vec3d    velocity;
        float    confidence;
    } ballStructure;
    typedef std::vector<ballStructure> ballList;

    typedef struct
    {
        vec2d    position;
        vec2d    velocity;
        float    radius;
        float    confidence;
    } obstacleStructure;
    typedef std::vector<obstacleStructure> obstacleList;

    typedef struct
    {
        std::string type;
        std::string teamName;
        std::string globalIntention;
        robotList robots;
        ballList balls;
        obstacleList obstacles;
        size_t ageMs;
    } packetStructureDeserialized;
}

#endif /* PACKETSTRUCTUREREFBOXLOGGER_HPP_ */
