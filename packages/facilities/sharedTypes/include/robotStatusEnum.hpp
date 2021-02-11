// Copyright 2018-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * robotStatusEnum.hpp
 *
 *  Created on: Jul 22, 2018
 *      Author: Jan Feitsma
 */

#ifndef ROBOTSTATUSENUM_HPP_
#define ROBOTSTATUSENUM_HPP_

#include "RtDB2.h" // required for serialization

enum class robotStatusEnum
{
    UNKNOWN,
    INITIALIZING, // before having a valid loc
    INPLAY,       // valid loc + inplay button = participating in team
    OUTOFPLAY     // button toggled off
};

SERIALIZE_ENUM(robotStatusEnum);

#endif

