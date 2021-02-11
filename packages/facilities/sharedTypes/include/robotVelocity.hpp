// Copyright 2018-2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotVelocity.hpp
 *
 *  Created on: Nov 6, 2018
 *      Author: Erik Kouters
 *
 */

#ifndef ROBOTVELOCITY_HPP_
#define ROBOTVELOCITY_HPP_

#include "pose.hpp"

#include "RtDB2.h" // required for serialization


// velocity in RCS has components (x,y,Rz), so we reuse pose
typedef pose robotVelocity;

#endif

