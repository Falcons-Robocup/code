// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef DIAGVELOCITYCONTROL_HPP_
#define DIAGVELOCITYCONTROL_HPP_

#include "pose.hpp"
#include "RtDB2.h" // required for serialization

struct diagVelocityControl
{
    pose                  accelerationRCS; // for kstplot_motion
    std::vector<bool>     isAccelerating;
    std::vector<bool>     accelerationClipping;
    bool                  shortStroke;
    std::vector<bool>     deadzone;

    pose currentPosition;
    pose currentVelocity;
    pose maxVelocity;
    pose maxAcceleration;
    pose targetPosition;
    pose targetVelocity;
    pose newPosition;
    pose newVelocity;

    SERIALIZE_DATA(accelerationRCS, isAccelerating, accelerationClipping, shortStroke, deadzone, currentPosition, currentVelocity, maxVelocity, maxAcceleration, targetPosition, targetVelocity, newPosition, newVelocity);
};

#endif

