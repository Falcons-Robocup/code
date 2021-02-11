// Copyright 2018-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * diagHealthFast.hpp
 *
 *  Created on: Dec 16, 2018
 *      Author: Jan Feitsma
 *
 */

#ifndef DIAGHEALTHFAST_HPP_
#define DIAGHEALTHFAST_HPP_

#include "RtDB2.h" // required for serialization

struct diagHealthFast
{
    float       cpuLoad;
    float       networkLoad;
    int         semCount;
    
    SERIALIZE_DATA(cpuLoad, networkLoad);
};

#endif

