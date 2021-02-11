// Copyright 2018-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * obstacleResult.hpp
 *
 *  Created on: Aug 14, 2018
 *      Author: Jan Feitsma
 *
 */

#ifndef OBSTACLERESULT_HPP_
#define OBSTACLERESULT_HPP_

#include "vec2d.hpp"

#include "RtDB2.h" // required for serialization


struct obstacleResult
{
    vec2d          position;
    vec2d          velocity;
    float          confidence = 0.0;
    int            id = 0; // optional, if we ever want to track/identify opponents

    SERIALIZE_DATA_FIXED(position, velocity, confidence, id);
};

#endif

