// Copyright 2018 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * shootSetpoint.hpp
 *
 *  Created on: Dec 08, 2018
 *      Author: Erik Kouters
 *
 */

#ifndef SHOOTSETPOINT_HPP_
#define SHOOTSETPOINT_HPP_

#include "shootPhaseEnum.hpp"
#include "shootTypeEnum.hpp"
#include "vec3d.hpp"

#include "RtDB2.h" // required for serialization


struct shootSetpoint
{
    shootPhaseEnum     shootPhase;
    shootTypeEnum      shootType;
    vec3d              position;
    
    SERIALIZE_DATA(shootPhase, shootType, position);
};

#endif

