// Copyright 2018-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * pose.hpp
 *
 *  Created on: Jul 22, 2018
 *      Author: Jan Feitsma
 */

#ifndef POSE_HPP_
#define POSE_HPP_

#include "RtDB2.h" // required for serialization

struct pose
{
    float x = 0;
    float y = 0;
    float Rz = 0;
    
    SERIALIZE_DATA_FIXED(x, y, Rz);

    pose(float xx = 0.0, float yy = 0.0, float Rzz = 0.0) : x(xx), y(yy), Rz(Rzz) {}

};


#endif

