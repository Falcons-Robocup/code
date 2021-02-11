// Copyright 2015 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cBallTypes.hpp
 *
 *  Created on: Dec 3, 2015
 *      Author: Coen Tempelaars
 */

#ifndef CBALLLOCATIONTYPES_HPP_
#define CBALLLOCATIONTYPES_HPP_

#include "vector3d.hpp"
#include <vector>


typedef struct
{
    Point3D position;
    Vector3D velocity;
    float confidence;
} ballLocation;

typedef std::vector<ballLocation> ballLocations;


#endif /* CBALLLOCATIONTYPES_HPP_ */
