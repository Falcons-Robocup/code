// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ppgeometry.hpp
 *
 *  Created on: December, 2019
 *      Author: Jan Feitsma
 */

#ifndef PPGEOMETRY_HPP_
#define PPGEOMETRY_HPP_

#include "falconsCommon.hpp"

Position2D addRcsToFcs(Position2D const &posRcs, Position2D const &posFcs);
Position2D faceTowards(Position2D const &current, float targetX, float targetY);

#endif

