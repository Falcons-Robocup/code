// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * vcgeometry.hpp
 *
 *  Created on: December, 2019
 *      Author: Jan Feitsma
 */

#ifndef VCGEOMETRY_HPP_
#define VCGEOMETRY_HPP_

#include "falconsCommon.hpp"

Position2D addRcsToFcs(Position2D const &posRcs, Position2D const &posFcs);
Position2D faceTowards(Position2D const &current, float targetX, float targetY);

#endif

