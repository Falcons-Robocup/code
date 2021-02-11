// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * CoordinateSystemEnum.hpp
 *
 *  Created on: Nov 2019
 *      Author: Jan Feitsma
 */

#ifndef COORDINATESYSTEMENUM_HPP_
#define COORDINATESYSTEMENUM_HPP_

#include "RtDB2.h" // required for serialization

enum class CoordinateSystemEnum
{
    RCS, // robot coordinate system (attached to robot center)
    FCS, // field coordinate system (playing forward towards +y)
    ACS  // absolute coordinate system (for simulation/visualization)
};

// see also wiki:CoordinateSystems

SERIALIZE_ENUM(CoordinateSystemEnum);

#endif

