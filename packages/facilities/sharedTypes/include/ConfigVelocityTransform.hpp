// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef CONFIGVELOCITYTRANSFORM_HPP_
#define CONFIGVELOCITYTRANSFORM_HPP_

#include "RtDB2.h" // required for serialization

struct ConfigVelocityTransform
{

    double      matrix[3][3];
    double      motorMaxVel;

    SERIALIZE_DATA(matrix, motorMaxVel);
};

#endif

