// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * behTreeReturnEnum.hpp
 *
 *  Created on: April 2019
 *      Author: Jan Feitsma
 *
 * (moved from package teamplay)
 */

#ifndef BEHTREERETURNENUM_HPP_
#define BEHTREERETURNENUM_HPP_

#include "RtDB2.h"

enum class behTreeReturnEnum
{
    INVALID = 0,

    PASSED,
    FAILED,
    RUNNING
};

SERIALIZE_ENUM(behTreeReturnEnum);

#endif

