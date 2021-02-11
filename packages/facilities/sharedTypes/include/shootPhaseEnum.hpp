// Copyright 2018-2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * shootPhaseEnum.hpp
 *
 *  Created on: Dec 08, 2018
 *      Author: Erik Kouters
 */

#ifndef SHOOTPHASEENUM_HPP_
#define SHOOTPHASEENUM_HPP_

#include "RtDB2.h" // required for serialization


enum class shootPhaseEnum
{
    NONE,
    PREPARE,
    SHOOT
};

SERIALIZE_ENUM(shootPhaseEnum);

#endif

