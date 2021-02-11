// Copyright 2017 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cShotCalibration.hpp
 *
 *  Created on: Dec 17, 2017
 *      Author: Jan Feitsma
 */

#ifndef CSHOTCALIBRATION_HPP_
#define CSHOTCALIBRATION_HPP_

#include <vector>
#include <tuple>

class cShotCalibration
{
  public:
    cShotCalibration();
    ~cShotCalibration();

    void load(std::string const &filename);
    void clear();
    void get(std::vector<std::tuple<float, float, float, float>> &measurementSamples);

  private:
    std::vector<std::tuple<float, float, float, float>> _measurements;
};

#endif /* CSHOTCALIBRATION_HPP_ */

