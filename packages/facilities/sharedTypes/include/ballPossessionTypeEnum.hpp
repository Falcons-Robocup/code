// Copyright 2018-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ballPossessionTypeEnum.hpp
 *
 *  Created on: Aug 14, 2018
 *      Author: Jan Feitsma
 */

#ifndef BALLPOSSESSIONTYPEENUM_HPP_
#define BALLPOSSESSIONTYPEENUM_HPP_

#include "RtDB2.h" // required for serialization


enum class ballPossessionTypeEnum
{
    UNKNOWN,
    FIELD,
    TEAM,
    OPPONENT
};

SERIALIZE_ENUM(ballPossessionTypeEnum);

#endif

