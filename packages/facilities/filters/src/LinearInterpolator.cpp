// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * LinearInterpolator.cpp
 *
 *  Created on: Dec 2019
 *      Author: Jan Feitsma
 */

#include "ext/LinearInterpolator.hpp"


float LinearInterpolator::evaluate(float x)
{
    // require at least 2 data points
    if (_data.size() < 2)
    {
        throw std::runtime_error("insufficient data");
    }

    // find segment (as 'low' and 'up' iterators)
    auto low = _data.begin();
    auto up = std::next(low);
    while ((x > up->first) && (std::next(up) != _data.end()))
    {
        ++low;
        ++up;
    }

    // linearize segment
    float dx = up->first - low->first; // cannot become zero due to using map
    float dy = up->second - low->second;

    // calculate
    return low->second + (x - low->first) * dy / dx;
}

