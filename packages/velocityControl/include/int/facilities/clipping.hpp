// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * clipping.hpp
 *
 *  Created on: Nov, 2019
 *      Author: Jan Feitsma
 */

#ifndef CLIPPING_HPP_
#define CLIPPING_HPP_

#include "falconsCommon.hpp"


// adapter for template clip()
inline double fabs(Vector2D const &v)
{
    return v.size();
}

// generalized clipping of a value v to an area around p with radius r
// v may be a float or vector2d
template <typename T>
bool gclip(T &v, T const p, float r, char const *label = "")
{
    r = fabs(r);
    T vo = v;
    T delta = v - p;
    if ((fabs(delta) > r) && (fabs(delta) > 0.1))
    {
        // normalize
        delta *= r / fabs(delta);
        v = p + delta;
        TRACE("clipping %s from %6.2f to %6.2f", label, fabs(vo), fabs(v));
        return true;
    }
    return false;
}

#endif

