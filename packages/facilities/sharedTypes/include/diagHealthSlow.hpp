// Copyright 2018 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * diagHealthSlow.hpp
 *
 *  Created on: Dec 16, 2018
 *      Author: Jan Feitsma
 *
 */

#ifndef DIAGHEALTHSLOW_HPP_
#define DIAGHEALTHSLOW_HPP_

#include "RtDB2.h" // required for serialization

struct diagHealthSlow
{
    int          diskUsage; // percentage: 0-100
    
    SERIALIZE_DATA(diskUsage);
};

#endif

