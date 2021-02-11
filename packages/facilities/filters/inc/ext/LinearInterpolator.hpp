// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * LinearInterpolator.hpp
 *
 *  Created on: Dec 2019
 *      Author: Jan Feitsma
 */

#ifndef _INCLUDED_LINEARINTERPOLATOR_HPP_
#define _INCLUDED_LINEARINTERPOLATOR_HPP_


#include "AbstractInterpolator.hpp"


class LinearInterpolator: public AbstractInterpolator
{
public:
    float evaluate(float x);
};

#endif

