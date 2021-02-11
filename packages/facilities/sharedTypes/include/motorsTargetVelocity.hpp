// Copyright 2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * motorsTargetVelocity.hpp
 *
 *  Created on: Jan 02, 2018
 *      Author: Erik Kouters
 *
 */

#ifndef MOTORSTARGETVELOCITY_HPP_
#define MOTORSTARGETVELOCITY_HPP_

#include "RtDB2.h" // required for serialization


typedef float motorTargetVelocity;

struct motorsTargetVelocity
{
    motorTargetVelocity   m1;
    motorTargetVelocity   m2;
    motorTargetVelocity   m3;
    
    SERIALIZE_DATA_FIXED(m1, m2, m3);
};

#endif

