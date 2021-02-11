// Copyright 2018-2019 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * forbiddenArea.hpp
 *
 *  Created on: Dec 01, 2018
 *      Author: Erik Kouters
 *
 */

#ifndef FORBIDDENAREA_HPP_
#define FORBIDDENAREA_HPP_

#include "polygon.hpp"

struct forbiddenArea: public polygon
{
    int id; // extra w.r.t. polygon

    SERIALIZE_DATA_FIXED(id, points);
};

#endif

