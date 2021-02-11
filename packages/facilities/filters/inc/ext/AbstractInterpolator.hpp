// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * AbstractInterpolator.hpp
 *
 *  Created on: Dec 2019
 *      Author: Jan Feitsma
 */

#ifndef _INCLUDED_ABSTRACTINTERPOLATOR_HPP_
#define _INCLUDED_ABSTRACTINTERPOLATOR_HPP_


#include <map>
#include <string>


class AbstractInterpolator
{
public:
    AbstractInterpolator();
    virtual ~AbstractInterpolator();

    // load data from file
    void load(std::string const &filename);

    // clear data
    void clear();

    // insert a data point
    void feed(double x, double y);

    // evaluate y at given x
    virtual float evaluate(float x) = 0;

    // write data and interpolation results to file, for visual inspection
    // (see tst/plotInterpolation.py)
    void writeSampleData(std::string const &filename = "/var/tmp/interpolationSampleData.txt");
    void writeInterpolatedCurve(float xmin, float xmax, float dx, std::string const &filename = "/var/tmp/interpolationCurve.txt");

    // data (x,y) pairs
    std::map<double, double> _data;
};

#endif

