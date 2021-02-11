// Copyright 2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * motorsFeedback.hpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Erik Kouters
 *
 */

#ifndef MOTORSFEEDBACK_HPP_
#define MOTORSFEEDBACK_HPP_

#include "RtDB2.h" // required for serialization

#include "motorDisplacementAndVelocity.hpp"


struct motorsFeedback
{
    motorDisplacementAndVelocity   m1;
    motorDisplacementAndVelocity   m2;
    motorDisplacementAndVelocity   m3;
    
    SERIALIZE_DATA_FIXED(m1, m2, m3);
};

#endif

