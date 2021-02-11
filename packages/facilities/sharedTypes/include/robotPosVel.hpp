// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotPosVel.hpp
 *
 *  Created on: Oct 3, 2020
 *      Author: Erik Kouters
 *
 */

#ifndef ROBOTPOSVEL_HPP_
#define ROBOTPOSVEL_HPP_

#include "pose.hpp"
#include "motionTypeEnum.hpp"

#include "RtDB2.h" // required for serialization

enum class robotPosVelEnum
{
    INVALID,
    POSVEL,     // have the target velocity at the target position
    POS_ONLY,   // equivalent to POSVEL with vel(0,0,0)
    VEL_ONLY    // velocity setpoint, ignoring the robot position. used to stop the robot: VEL_ONLY with vel(0,0,0).
};

SERIALIZE_ENUM(robotPosVelEnum);

struct robotPosVel
{

    // robotPosVelType defines which inputs are valid. (pos+vel, only pos, only vel)
    // stop = VEL_ONLY && vel(0, 0, 0)
    robotPosVelEnum    robotPosVelType;

    pose               position;

    // velocity in RCS has components (x,y,Rz), so we reuse pose
    pose               velocity;

    motionTypeEnum     motionType; // different move types (e.g., normal, accurate (setpiece), intercept)
    
    SERIALIZE_DATA_FIXED(robotPosVelType, position, velocity, motionType);
};

#endif

