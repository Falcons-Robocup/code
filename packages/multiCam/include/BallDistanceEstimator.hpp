// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * BallDistanceEstimator.hpp
 *
 *  Created on: Dec 2019
 *      Author: Jan Feitsma
 */

#ifndef _INCLUDED_BALLDISTANCEESTIMATOR_HPP_
#define _INCLUDED_BALLDISTANCEESTIMATOR_HPP_


#include "LeastSquaresInterpolator.hpp" // from package 'filters'



class BallDistanceEstimator: public LeastSquaresInterpolator
{
public:
    BallDistanceEstimator(float fitRange = 200.0, float resolution = 1.0, int order = 2);

    // pixel-to-distance conversion formula
    float radiusPixelsMmFormula(float pixels); // output in mm
    float estimatedFormula(float pixels); // output in m

    // produce a best guess for distance
    float evaluate(float numPixels);

    // feed the estimator with a new data point
    void feed(float numPixels, float distance);

private:
    float _fitRange;
    float _resolution;
};

#endif

