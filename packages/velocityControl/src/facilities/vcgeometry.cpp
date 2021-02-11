// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * vcgeometry.cpp
 *
 *  Created on: December, 2019
 *      Author: Jan Feitsma
 */

#include "int/facilities/vcgeometry.hpp"


Position2D addRcsToFcs(Position2D const &posRcs, Position2D const &posFcs)
{
    // return posFcs with posRcs offset added
    // TODO (#14): awkward old Position2D API ... we should actually improve the core Position2D class ...
    Position2D result = posRcs;
    result.transform_rcs2fcs(posFcs);
    result.phi = posFcs.phi;
    return result;
}

Position2D faceTowards(Position2D const &current, float targetX, float targetY)
{
    Position2D result = current;
    result.phi = atan2(targetY - current.y, targetX - current.x);
    return result;
}

