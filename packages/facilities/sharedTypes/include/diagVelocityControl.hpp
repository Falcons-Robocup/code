// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef DIAGVELOCITYCONTROL_HPP_
#define DIAGVELOCITYCONTROL_HPP_

#include "pose.hpp"
#include "PIDTerms.hpp"

#include "RtDB2.h" // required for serialization


struct diagPIDstate
{
    PIDTerms x;
    PIDTerms y;
    PIDTerms Rz;
    SERIALIZE_DATA(x, y, Rz);
};


struct diagVelocityControl
{
    pose                  accelerationRCS; // for kstplot_motion
    std::vector<bool>     isAccelerating;
    std::vector<bool>     accelerationClipping;
    bool                  shortStroke;
    std::vector<bool>     deadzone;
    diagPIDstate          pid;

    SERIALIZE_DATA(accelerationRCS, isAccelerating, accelerationClipping, shortStroke, deadzone, pid);
};

#endif

