// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ftime.hpp
 *
 * Falcons timestamp utilities, allowing for simulated time
 *
 *  Created on: June, 2020
 *      Author: Erik Kouters
 */

#ifndef FTIME_HPP_
#define FTIME_HPP_

#include <string>
#include "FalconsRTDB.hpp"

class ftime
{
public:

    // get "current" timestamp -- can be simulated time
    // Usage: ftime::now()
    static rtime now();

};

#endif
