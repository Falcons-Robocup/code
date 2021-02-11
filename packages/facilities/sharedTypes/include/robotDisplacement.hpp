// Copyright 2018-2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotDisplacement.hpp
 *
 *  Created on: Nov 6, 2018
 *      Author: Erik Kouters
 *
 */

#ifndef ROBOTDISPLACEMENT_HPP_
#define ROBOTDISPLACEMENT_HPP_

#include "vec3d.hpp"

#include "RtDB2.h" // required for serialization


// displacement in RCS has components (x,y,Rz), so we reuse pose
typedef pose robotDisplacement;


#endif

