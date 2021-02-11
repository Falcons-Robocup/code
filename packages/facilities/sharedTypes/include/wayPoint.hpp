// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef WAYPOINT_HPP_
#define WAYPOINT_HPP_

#include "pose.hpp"

#include "RtDB2.h" // required for serialization


struct wayPoint
{
    pose pos;
    pose vel;
    SERIALIZE_DATA_FIXED(pos, vel);
};

#endif

