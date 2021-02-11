// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * intercept.hpp
 *
 *  Created on: Mar 28, 2019
 *      Author: Jan Feitsma
 */

#ifndef INTERCEPT_HPP_
#define INTERCEPT_HPP_


#include "falconsCommon.hpp"


void calculateIntercept(
    // inputs
    Position2D const &robotPos, 
    Velocity2D const &robotVel, // ignore for now, would need to call spg
    Vector3D const &objectPos, 
    Vector3D const &objectVel, // assume object moves at constant position
    // outputs
    bool &success,
    float &timeNeeded,
    Position2D &targetPos,
    Velocity2D &targetVel);
    

#endif

