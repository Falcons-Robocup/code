// Copyright 2018-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ballResult.hpp
 *
 *  Created on: Aug 14, 2018
 *      Author: Jan Feitsma
 *
 */

#ifndef BALLRESULT_HPP_
#define BALLRESULT_HPP_

#include "ballPossession.hpp"
#include "vec3d.hpp"

#include "RtDB2.h" // required for serialization


struct ballResult
{
    vec3d          position;
    vec3d          velocity;
    float          confidence;
    ballPossession owner;
    
    SERIALIZE_DATA_FIXED(position, velocity, confidence, owner);
};

#endif

