// Copyright 2018-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * vec2d.hpp
 *
 *  Created on: Aug 14, 2018
 *      Author: Jan Feitsma
 *
 */

#ifndef VEC2D_HPP_
#define VEC2D_HPP_

#include "RtDB2.h" // required for serialization


struct vec2d
{
    float      x = 0.0;
    float      y = 0.0;
    
    vec2d(float xx = 0.0, float yy = 0.0)
    {
        x = xx;
        y = yy;
    }
    
    SERIALIZE_DATA_FIXED(x, y);
};

#endif

