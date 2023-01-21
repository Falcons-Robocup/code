// Copyright 2019-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * tpOverrideLevelEnum.hpp
 *
 *  Created on: April 2019
 *      Author: Jan Feitsma
 */

#ifndef TPOVERRIDELEVELENUM_HPP_
#define TPOVERRIDELEVELENUM_HPP_

#include "RtDB2.h"

enum class tpOverrideLevelEnum
{
    INVALID,
    GAMESTATE,
    ROLE
};

SERIALIZE_ENUM(tpOverrideLevelEnum);

#endif

