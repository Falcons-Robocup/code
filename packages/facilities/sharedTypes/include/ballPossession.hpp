// Copyright 2018 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ballPossession.hpp
 *
 *  Created on: Aug 14, 2018
 *      Author: Jan Feitsma
 *
 */

#ifndef BALLPOSSESSION_HPP_
#define BALLPOSSESSION_HPP_

#include "ballPossessionTypeEnum.hpp"

#include "RtDB2.h" // required for serialization


struct ballPossession
{
    ballPossessionTypeEnum type;
    int                    robotId; // can be friendly or even opponent robot id (roadmap)
    
    SERIALIZE_DATA_FIXED(type, robotId);
};

#endif

