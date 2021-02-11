// Copyright 2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * BallDistanceEstimator.cpp
 *
 *  Created on: January 2020
 *      Author: Jan Feitsma
 */


#include "BallDistanceEstimator.hpp"
#include <cmath>
#include <mutex>

#ifndef NOROS
#include "tracing.hpp"
#endif

std::mutex g_mutex_mc;



BallDistanceEstimator::BallDistanceEstimator(float fitRange, float resolution, int order)
    : LeastSquaresInterpolator(order, fitRange),
    _fitRange(fitRange),
    _resolution(resolution)
{
#ifndef NOROS
    TRACE_FUNCTION("construct");
#endif
    // set initial guess values based on legacy formula
    // these will be overwritten runtime as newer/better data comes in
    // note: we use a sparse but complete covering resolution here,
    // to make it easier for new data to overrule the inaccurate initial data,
    // while making sure to not starting empty handed
    float p = 0;
    p = 5; feed(p, estimatedFormula(p));
    p = 10; feed(p, estimatedFormula(p));
    p = 25; feed(p, estimatedFormula(p));
    for (p = 50; p < 500; p += 50)
    {
        feed(p, estimatedFormula(p));
    }
    p = 1000; feed(p, estimatedFormula(p));
    p = 5000; feed(p, estimatedFormula(p));
    p = 10000; feed(p, estimatedFormula(p));
}

float BallDistanceEstimator::estimatedFormula(float pixels)
{
    // updated estimation formula based on measurement (2020-02-25)
    return 71.0 / pixels;
}

// legacy pixel-to-distance conversion formula
float BallDistanceEstimator::radiusPixelsMmFormula(float pixels)
{
    // WARNING: output is in mm, not m
    return -136.3814 + 24133.4514 / (1 + pow((pixels / 8.401286), 0.5264725));
}

float BallDistanceEstimator::evaluate(float numPixels)
{
#ifndef NOROS
    TRACE_FUNCTION("");
#endif
    g_mutex_mc.lock();
    float result = 1.0;
    bool fallback = false;
    // use (weighted) interpolation
    try
    {
        result = LeastSquaresInterpolator::evaluate(numPixels);
    }
    catch (...)
    {
        fallback = true;
    }
    if (result < 0.1)
    {
        fallback = true;
    }
    if (fallback)
    {
#ifndef NOROS
        TRACE("fallback: numPixels=%.1f LSQresult=%.1f fallback=%.1f", numPixels, result, estimatedFormula(numPixels));
#endif
        result = estimatedFormula(numPixels);
    }
    g_mutex_mc.unlock();
    return result;
}

void BallDistanceEstimator::feed(float numPixels, float distance)
{
#ifndef NOROS
    TRACE_FUNCTION("");
#endif
    g_mutex_mc.lock();
    // round to configured resolution
    // (this can be used to control the sample size)
    float numPixelsRounded = round(numPixels / _resolution) * _resolution;
    // store
#ifndef NOROS
    TRACE("numPixels=%.1f numPixelsRounded=%.1f distance=%.1f", numPixels, numPixelsRounded, distance);
#endif
    _data[numPixelsRounded] = distance;
    g_mutex_mc.unlock();
}

