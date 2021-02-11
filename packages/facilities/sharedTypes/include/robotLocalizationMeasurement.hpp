// Copyright 2018-2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotLocationMeasurement.hpp
 *
 *  Created on: Oct 20, 2018
 *      Author: Erik Kouters
 *
 */

#ifndef ROBOTLOCALIZATIONMEASUREMENT_HPP_
#define ROBOTLOCALIZATIONMEASUREMENT_HPP_

#include "uniqueObjectID.hpp"
#include "RtDB2.h" // required for serialization

struct robotLocalizationMeasurement
{
    uniqueObjectID     identifier;
    rtime              timestamp; // instead of timeval, for performance and ease of computation
    float              x;
    float              y;
    float              theta;
    float              confidence;
    
    SERIALIZE_DATA_FIXED(identifier, timestamp, x, y, theta, confidence);
};

#endif

