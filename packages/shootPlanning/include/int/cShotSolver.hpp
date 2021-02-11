// Copyright 2017-2018 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cShotSolver.hpp
 *
 *  Created on: Dec 17, 2017
 *      Author: Jan Feitsma
 */

#ifndef CSHOTSOLVER_HPP_
#define CSHOTSOLVER_HPP_

#include <string>
#include <map>
#include <tuple>
#include "int/cShotCalibration.hpp"


class cShotSolver
{
    public:
        cShotSolver();
        ~cShotSolver();

        void load(std::string const &filename);

        bool query(float distance, float z, bool doLob, float &resultStrength, float &resultAngle);

    private:
        cShotCalibration _calibration;
        std::map<float, std::tuple<float, float>> _lookupTblStraight;  //  angle -> (max_z, max_distance) , power = 180
        std::map<float, std::tuple<float, float>> _lookupTblLob;       // power -> (max_z, max_distance) , angle = 180
        
        void selectShots();
};

#endif /* CSHOTSOLVER_HPP_ */

