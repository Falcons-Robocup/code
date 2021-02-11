// Copyright 2017 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cShotCalibration.cpp
 *
 *  Created on: Dec 17, 2017
 *      Author: Jan Feitsma
 */

#include "int/cShotCalibration.hpp"
#include <fstream>
#include <sstream>


cShotCalibration::cShotCalibration()
{
}

cShotCalibration::~cShotCalibration()
{
}

void cShotCalibration::load(std::string const &filename)
{
    // load measurement samples
    std::ifstream infile(filename.c_str());
    std::string line;
    while (std::getline(infile, line))
    {
        if (line.size() == 0) continue;
        if (line[0] == '#') continue;
        std::istringstream iss(line);
        float a, b, c, d;
        if (!(iss >> a >> b >> c >> d)) break; // error
        // process line data
        _measurements.push_back(std::tuple<float, float, float, float>(a, b, c, d));
    }
}

void cShotCalibration::get(std::vector<std::tuple<float, float, float, float>> &measurements)
{
    measurements = _measurements;
}

void cShotCalibration::clear()
{
    _measurements.clear();
}


