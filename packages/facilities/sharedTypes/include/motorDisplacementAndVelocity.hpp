// Copyright 2018-2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * motorDisplacementAndVelocity.hpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Erik Kouters
 *
 */

#ifndef MOTORDISPLACEMENTANDVELOCITY_HPP_
#define MOTORDISPLACEMENTANDVELOCITY_HPP_

#include "RtDB2.h" // required for serialization


struct motorDisplacementAndVelocity
{
    float               displacement;
    float               velocity;
    
    SERIALIZE_DATA_FIXED(displacement, velocity);
};

#endif

