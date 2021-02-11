// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * AbstractInterpolator.cpp
 *
 *  Created on: Dec 2019
 *      Author: Jan Feitsma
 */

#include "ext/AbstractInterpolator.hpp"

#include <fstream>
#include <sstream>
#include <iomanip>


AbstractInterpolator::AbstractInterpolator()
{
}

AbstractInterpolator::~AbstractInterpolator()
{
}

void AbstractInterpolator::load(std::string const &filename)
{
    std::ifstream infile(filename.c_str());
    std::string line;
    while (std::getline(infile, line))
    {
        // ignore empty lines
        if (line.size() == 0) continue;
        // allow #comments lines
        if (line[0] == '#') continue; 
        std::istringstream iss(line);
        float a, b;
        if (!(iss >> a >> b)) break; // error
        // process pair (a,b)
        feed(a, b);
    }
}

void AbstractInterpolator::clear()
{
    _data.clear();
}

void AbstractInterpolator::feed(double x, double y)
{
    _data[x] = y;
}

void AbstractInterpolator::writeSampleData(std::string const &filename)
{
    std::ofstream outfile(filename.c_str());
    for (auto it = _data.begin(); it != _data.end(); ++it)
    {
        float x = it->first;
        float y = it->second;
        outfile << std::fixed << std::setw(8) << std::setprecision(3) << x << " ";
        outfile << std::fixed << std::setw(8) << std::setprecision(3) << y << std::endl;
    }
}

void AbstractInterpolator::writeInterpolatedCurve(float xmin, float xmax, float dx, std::string const &filename)
{
    std::ofstream outfile(filename.c_str());
    for (float x = xmin; x <= xmax; x += dx)
    {
        float y = evaluate(x);
        outfile << std::fixed << std::setw(8) << std::setprecision(3) << x << " ";
        outfile << std::fixed << std::setw(8) << std::setprecision(3) << y << std::endl;
    }
}

