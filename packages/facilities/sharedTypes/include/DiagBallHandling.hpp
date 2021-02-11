// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef DIAGBALLHANDLING_HPP_
#define DIAGBALLHANDLING_HPP_

#include "RtDB2.h" // required for serialization


struct BallHandlingArmDiag
{
    bool enabled = false;
    bool lifted = false;
    float angleFraction = 0.0;
    float velocitySetpoint = 0.0;
    SERIALIZE_DATA(enabled, lifted, angleFraction, velocitySetpoint);
};

struct DiagBallHandling
{
    bool ballPossession = false;
    BallHandlingArmDiag left;
    BallHandlingArmDiag right;
    SERIALIZE_DATA(ballPossession, left, right);
};

#endif

