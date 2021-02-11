// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * LeastSquaresInterpolator.cpp
 *
 *  Created on: Dec 2019
 *      Author: Jan Feitsma
 */

#include "ext/LeastSquaresInterpolator.hpp"
#include "ext/LinearInterpolator.hpp"

#include <libalglib/interpolation.h>
#include <libalglib/specialfunctions.h>


LeastSquaresInterpolator::LeastSquaresInterpolator(int order, float range)
{
    _order = order;
    _range = range;
}

// standard probability density function
// belonging to normal distribution
// (annoyingly enough, it was chosen to not make this a part of alglib nor stdlib ?!!)
// https://stackoverflow.com/questions/19387222/probability-density-function-using-the-standard-library
float pdf(float x)
{
    return exp(-x*x*0.5) / sqrt(2*M_PI);
}

float LeastSquaresInterpolator::evaluate(float x)
{
    // use alglib to calculate polynomial, while selecting data of interest
    // see also alglib example lsfit_d_pol.c

    // require at least 1 data point
    if (_data.size() < 1)
    {
        throw std::runtime_error("no data to fit");
    }

    // construct alglib arrays
    // for continuity, weights are chosen as function of distance (scaled with configured range)
    alglib::real_1d_array w;
    alglib::real_1d_array xa;
    alglib::real_1d_array ya;
    w.setlength(_data.size());
    xa.setlength(_data.size());
    ya.setlength(_data.size());
    size_t idx = 0;
    float totalWeight = 0.0;
    for (auto it = _data.begin(); it != _data.end(); ++it)
    {
        float xx = it->first;
        xa[idx] = xx;
        ya[idx] = it->second;
        w[idx] = pdf((xx - x) / _range);
        totalWeight += w[idx];
        idx++;
    }

    // fallback to linear interpolation
    if (totalWeight < 0.01)
    {
        LinearInterpolator fallbackInterpolator;
        fallbackInterpolator._data = _data;
        return fallbackInterpolator.evaluate(x);
    }

    // (weighted) fit polynomial
    alglib::ae_int_t m = _order + 1;
    alglib::ae_int_t info;
    alglib::barycentricinterpolant p;
    alglib::polynomialfitreport rep;
    alglib::real_1d_array xc = "[]";
    alglib::real_1d_array yc = "[]";
    alglib::integer_1d_array dc = "[]";
    polynomialfitwc(xa, ya, w, xc, yc, dc, m, info, p, rep);

    // evaluate at given x
    return barycentriccalc(p, x);
}

std::vector<double> LeastSquaresInterpolator::calculate_polynomial()
{
    alglib::real_1d_array xa;
    alglib::real_1d_array ya;
    xa.setlength(_data.size());
    ya.setlength(_data.size());
    size_t idx = 0;    

    for (auto it = _data.begin(); it != _data.end(); ++it)
    {
        float xx = it->first;
        xa[idx] = xx;
        ya[idx] = it->second;
        idx++;
    }

    alglib::ae_int_t m = _order + 1;
    alglib::ae_int_t info;
    alglib::barycentricinterpolant p;
    alglib::polynomialfitreport rep;
    alglib::real_1d_array coeff;
    polynomialfit(xa, ya, m, info, p, rep);
    polynomialbar2pow(p, coeff);

    std::vector<double> result;

    for(int i=0; i<coeff.length(); i++)
    {
        result.push_back(coeff[i]);
    }
    return result;
}
