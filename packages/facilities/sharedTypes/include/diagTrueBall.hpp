// Copyright 2020 lucas (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * diagTrueBall.hpp
 *
 *  Created on: Aug, 2020
 *      Author: Lucas Catabriga
 *
 */

#ifndef DIAGTRUEBALL_HPP_
#define DIAGTRUEBALL_HPP_

#include "RtDB2.h" // required for serialization

struct diagTrueBall
{
    double x;
    double y;
    double z;
    rtime timestamp;

    SERIALIZE_DATA(x, y, z, timestamp);
};

#endif

