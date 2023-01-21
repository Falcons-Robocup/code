// Copyright 2019-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionInterceptBall.hpp
 *
 *  Created on: Apr 23, 2018
 *      Author: Erik Kouters
 */

#ifndef MP_ACTIONINTERCEPTBALL_HPP_
#define MP_ACTIONINTERCEPTBALL_HPP_

#include <boost/optional.hpp>

#include "MP_AbstractAction.hpp"

class MP_ActionInterceptBall: public MP_AbstractAction
{
public:
    actionResultTypeEnum execute();
    void unpackParameters();

    void initialize() override;

private:
    bool _ballMovingTowardsOurFront = false;
    bool _ballMovingTowardsUs = false;
    bool _ballMovingFastEnough = false;
    float _obstacleThreshold  = 3.0;
    float _ballSpeedThreshold = 0.35; // 0.35 was the original setting in teamplay getBallOnVector
    float _ballSpeedScaling   = 0.6; // 0.6 was the original setting in teamplay getBallOnVector, although it might have been defunct according to Erik
    float _captureRadius      = 2.1; // 2.1 was the original setting in teamplay interceptBall
    bool  _activeIntercept    = false; // false was the original setting in teamplay interceptBall
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
    
    boost::optional<Position2D> positionIntercept();
    boost::optional<Position2D> activePositionIntercept();
    bool isBallMovingTowardsOurFront();
    bool isBallMovingTowardsUs();
    bool isBallMovingFastEnough();
};

#endif /* MP_ACTIONiNTERCEPTBALL_HPP_ */

