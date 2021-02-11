// Copyright 2018-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * motionSetpoint.hpp
 *
 *  Created on: Dec 8, 2018
 *      Author: Erik Kouters
 *
 */

#ifndef MOTIONSETPOINT_HPP_
#define MOTIONSETPOINT_HPP_

#include "actionTypeEnum.hpp"
#include "motionTypeEnum.hpp"
#include "vec3d.hpp"

#include "RtDB2.h" // required for serialization


struct motionSetpoint
{
    actionTypeEnum     action;
    vec3d              position; // could be interpreted as a pose (in case of move) or vec3d (when shooting)
    motionTypeEnum     motionType; // different move types (e.g., normal, accurate (setpiece), intercept)

    SERIALIZE_DATA(action, position, motionType);
    // we might extend this (for instance with target velocity) - dont use SERIALIZE_DATA_FIXED
};

#endif

