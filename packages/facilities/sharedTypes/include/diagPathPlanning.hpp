// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef DIAGPATHPLANNING_HPP_
#define DIAGPATHPLANNING_HPP_

#include "wayPoint.hpp"
#include "forbiddenArea.hpp"

#include "RtDB2.h" // required for serialization


struct diagPathPlanning
{
    std::vector<wayPoint> path; // can contain a single target, or no target, or even an extra intermediate (sub-)target
    std::vector<forbiddenArea> forbiddenAreas;
    pose                  distanceToSubTargetRCS; // for kstplot_motion
    int                   numCalculatedObstacles;

    SERIALIZE_DATA(path, forbiddenAreas, distanceToSubTargetRCS, numCalculatedObstacles);
};

#endif

