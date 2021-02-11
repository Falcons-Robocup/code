// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef CONFIGMOTIONPLANNING_HPP_
#define CONFIGMOTIONPLANNING_HPP_

#include "RtDB2.h" // required for serialization



struct GetBallConfig
{
    float obstacleThreshold = 3.0;
    float ballSpeedThreshold = 0.35;
    float ballSpeedScaling = 0.6;

    SERIALIZE_DATA(obstacleThreshold, ballSpeedThreshold, ballSpeedScaling);
};

struct InterceptBallConfig
{
    float obstacleThreshold = 3.0;
    float ballSpeedThreshold = 0.35;
    float ballSpeedScaling = 0.6;
    float captureRadius = 2.1;
    bool activeIntercept = false;

    SERIALIZE_DATA(obstacleThreshold, ballSpeedThreshold, ballSpeedScaling, captureRadius, activeIntercept);
};

struct KeeperMoveConfig
{
    float yMaxOffset = 0.3; // distance "out of the goal"
    float xGoalpostOffset = 0.45; // distance from the goalpost (sideways/X direction)

    SERIALIZE_DATA(yMaxOffset, xGoalpostOffset);
};

struct PassToTargetConfig
{
    float aimSettleTime = 0.2;
    float accuracy = 0.05; // keep trying, optimize for short pass
    float timeout = 1.0;
    float coarseAngle = 0.5;
    float disableBhDelay = 0.1; // for (short) passes, if tuned agressively, ballHandling will cause either small bump or reduce pass power
    float sleepAfterShoot = 0.3;

    SERIALIZE_DATA(aimSettleTime, accuracy, timeout, coarseAngle, disableBhDelay, sleepAfterShoot);
};

struct ShootAtTargetConfig
{
    float aimSettleTime = 0.1;
    float accuracy = 0.2;
    float timeout = 0.6;
    float coarseAngle = 0.5;
    float disableBhDelay = 0.0; 
    float sleepAfterShoot = 0.3;

    SERIALIZE_DATA(aimSettleTime, accuracy, timeout, coarseAngle, disableBhDelay, sleepAfterShoot);
};


struct ConfigMotionPlanning
{
    GetBallConfig           getBallConfig;
    InterceptBallConfig     interceptBallConfig;
    KeeperMoveConfig        keeperMoveConfig;
    PassToTargetConfig      passToTargetConfig;
    ShootAtTargetConfig     shootAtTargetConfig;

    SERIALIZE_DATA(getBallConfig, interceptBallConfig, keeperMoveConfig, passToTargetConfig, shootAtTargetConfig);
};

#endif

