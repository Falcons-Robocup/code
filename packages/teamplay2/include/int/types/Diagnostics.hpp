// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Diagnostics.hpp
 *
 *  Created on: Jul 13, 2017
 *      Author: Coen Tempelaars
 */

#ifndef DIAGNOSTICS_HPP_
#define DIAGNOSTICS_HPP_

#include <string.h>
#include "vector2d.hpp"
#include "HeightmapNames.hpp"

namespace teamplay
{

struct Diagnostics {
    bool aiming = false;
    Point2D shootTarget;
    std::string action;
    CompositeHeightmapName activeHeightmap = CompositeHeightmapName::INVALID;
};


} /* namespace teamplay */

#endif /* DIAGNOSTICS_HPP_ */
