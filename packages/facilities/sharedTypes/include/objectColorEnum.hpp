// Copyright 2018-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * objectColorEnum.hpp
 *
 *  Created on: Jul 9, 2018
 *      Author: Jan Feitsma
 */

#ifndef OBJECTCOLORENUM_HPP_
#define OBJECTCOLORENUM_HPP_

#include "RtDB2.h" // required for serialization

enum class objectColorEnum
{
    UNKNOWN,
    YELLOW,
    CYAN,
    MAGENTA,
    BLACK,
    BLACKWHITE // for black-and-white ball detection
};

SERIALIZE_ENUM(objectColorEnum);

#endif
