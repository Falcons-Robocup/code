// Copyright 2018 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * vec3d.hpp
 *
 *  Created on: Aug 14, 2018
 *      Author: Jan Feitsma
 *
 */

#ifndef VEC3D_HPP_
#define VEC3D_HPP_

#include "RtDB2.h" // required for serialization


struct vec3d
{
    float      x;
    float      y;
    float      z;
    
    SERIALIZE_DATA_FIXED(x, y, z);
};

#endif

