// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * PIDController.hpp
 *
 *  Created on: October, 2020
 *      Author: Erik Kouters
 */

#ifndef VELOCITYCONTROL_PIDCONTROLLER_HPP_
#define VELOCITYCONTROL_PIDCONTROLLER_HPP_



#include "PIDTerms.hpp" // sharedTypes


struct PIDParams
{
    float dt = 0.1;
    float P = 0.0;
    float I = 0.0;
    float D = 0.0;
    float maxI = 0.0;
    float fadeI = 0.99;
};


class PIDController
{

public:
    PIDController() {}
    ~PIDController() {}

    PIDParams params;
    PIDTerms terms;

    float calculate(float delta)
    {
        _integral = _integral * params.fadeI + delta * params.dt;
        if (_integral > params.maxI) _integral = params.maxI;
        if (_integral < -params.maxI) _integral = -params.maxI;
        float derivative = (delta - _previousDelta) / params.dt;
        _previousDelta = delta;
        // store terms for diagnostics
        terms.proportional = params.P * delta;
        terms.integral = params.I * _integral;
        terms.derivative = params.D * derivative;
        // all three add up to result
        float result = terms.proportional + terms.integral + terms.derivative;
        return result;
    }

private:
    float _integral = 0.0;
    float _previousDelta = 0.0;
};

#endif

