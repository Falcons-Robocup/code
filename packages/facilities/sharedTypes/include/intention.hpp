// Copyright 2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * intention.hpp
 *
 *  Created on: March 21, 2019
 *      Author: Coen Tempelaars
 *
 */

#ifndef INTENTION_HPP_
#define INTENTION_HPP_

#include "actionTypeEnum.hpp"
#include "vec3d.hpp"

#include "RtDB2.h" // required for serialization


struct intention
{
    actionTypeEnum     action;
    vec3d              position;

    SERIALIZE_DATA_FIXED(action, position);
};

#endif
