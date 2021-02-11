// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * motionTypeEnum.hpp
 *
 *  Created on: Nov 08, 2020
 *      Author: Erik Kouters
 */

#ifndef MOTIONTYPEENUM_HPP_
#define MOTIONTYPEENUM_HPP_

#include "RtDB2.h" // required for serialization

enum class motionTypeEnum
{
    INVALID,
    NORMAL,         // Default movement, as fast as possible without taking risks / bumping into obstacles
    WITH_BALL,      // Default movement with ball, minimizing risk to lose the ball
    ACCURATE,       // Sacrificing speed for accuracy, typically used in a setpiece
    INTERCEPT,      // Moving sideways, limit rotation for highest chance of catching ball
    SLOW,           // Slow movement for safety. Used for park. At the end of a match, when robots are parking, people often walk over the field between the robots.
    SPRINT          // Higher speed, higher risk, sacrificing accuracy. May have overshoot and bump into obstacles. Do not use when near obstacles.
};

SERIALIZE_ENUM(motionTypeEnum);

#endif

