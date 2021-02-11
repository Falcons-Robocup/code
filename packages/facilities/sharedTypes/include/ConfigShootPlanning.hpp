// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef CONFIGSHOOTPLANNING_HPP_
#define CONFIGSHOOTPLANNING_HPP_

#include "RtDB2.h" // required for serialization


struct ConfigShootPlanning
{
    bool alwaysLobshot = false;
    bool alwaysStraightshot = true;
    float shotStrengthScaling = 1.0;
    float shotAngleScaling = 0.7;
    float lobAngleScaling = 0.5;
    float passScaling = 1.0;
    float minLobDistance = 6.0;
    float maxLobDistance = 9.0;

    SERIALIZE_DATA(alwaysLobshot, alwaysStraightshot, shotStrengthScaling, shotAngleScaling, lobAngleScaling, passScaling, minLobDistance, maxLobDistance);
};

#endif

