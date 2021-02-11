// Copyright 2018 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * shootTypeEnum.hpp
 *
 *  Created on: Dec 08, 2018
 *      Author: Erik Kouters
 */

#ifndef SHOOTTYPEENUM_HPP_
#define SHOOTTYPEENUM_HPP_

#include "RtDB2.h" // required for serialization


enum class shootTypeEnum
{
   PASS,
   SHOOT,
   LOB
};

SERIALIZE_ENUM(shootTypeEnum);

#endif

