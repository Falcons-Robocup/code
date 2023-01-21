// Copyright 2019-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionGetBall.hpp
 *
 *  Created on: Feb 3, 2018
 *      Author: Jan Feitsma
 */

#ifndef MP_ACTIONGETBALL_HPP_
#define MP_ACTIONGETBALL_HPP_

#include "MP_AbstractAction.hpp"

class MP_ActionGetBall: public MP_AbstractAction
{
public:
    actionResultTypeEnum execute();
    void unpackParameters();

    void initialize() override;

private:
    motionTypeEnum _motionType = motionTypeEnum::INVALID;
    bool _ballMovingFastEnough = false;
    float _obstacleThreshold  = 0.0;
    float _ballSpeedThreshold = 0.0;
    float _ballSpeedScaling   = 0.0;
    Position2D _target; // intercept target
    Position2D _R;      // robot position
    Velocity2D _Vr;     // robot velocity
    Vector3D   _B;      // ball position
    Vector3D   _Vb;     // ball velocity
    
    // helper functions
    void getCfg();
    void analyzeGeometry();
    bool shouldIntercept();
    void faceBall();
    
};

#endif /* MP_ACTIONGETBALL_HPP_ */

