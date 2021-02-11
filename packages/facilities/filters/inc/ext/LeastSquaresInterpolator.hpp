// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * LeastSquaresInterpolator.hpp
 *
 *  Created on: Dec 2019
 *      Author: Jan Feitsma
 */

#ifndef _INCLUDED_LEASTSQUARESINTERPOLATOR_HPP_
#define _INCLUDED_LEASTSQUARESINTERPOLATOR_HPP_

#include <vector>

#include "AbstractInterpolator.hpp"


class LeastSquaresInterpolator: public AbstractInterpolator
{
public:
    LeastSquaresInterpolator(int order=1, float range=1e99);

    float evaluate(float x);
    std::vector<double> calculate_polynomial();

private:
    int _order;
    float _range;
};

#endif

