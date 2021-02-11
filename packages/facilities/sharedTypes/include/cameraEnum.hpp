// Copyright 2018-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cameraEnum.hpp
 *
 *  Created on: Sep 6, 2016
 *      Author: Tim Kouters
 */

#ifndef CAMERAENUM_HPP_
#define CAMERAENUM_HPP_

#include "RtDB2.h" // required for serialization

enum class cameraEnum
{
    INVALID,
    OMNIVISION,
    FRONTCAMERA,
    MULTICAMERA // no need to distinguish which one
};

SERIALIZE_ENUM(cameraEnum);

#endif /* CAMERAENUM_HPP_ */
