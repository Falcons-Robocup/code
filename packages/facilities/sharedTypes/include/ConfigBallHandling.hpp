// Copyright 2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef CONFIGBALLHANDLING_HPP_
#define CONFIGBALLHANDLING_HPP_

#include "RtDB2.h" // required for serialization



struct BallHandlingArmConfig
{
    float down = 2500; // sensor value related to down position
    float up = 4000; // sensor value related to up position

    SERIALIZE_DATA(down, up);
};

struct BallHandlingRobotConfig
{
    int robotId = 0;
    BallHandlingArmConfig leftArm;
    BallHandlingArmConfig rightArm;

    SERIALIZE_DATA(robotId, leftArm, rightArm);
};

struct BallPossessionConfig
{
    float angleThresholdOn = 0.4; // angle where ball is seen as detected
    float angleThresholdOff = 0.4; // angle where ball is seen as no longer detected
    float minimumTimeUp = 0.2; // time interval in seconds when ball possession is true after both arms go up

    SERIALIZE_DATA(angleThresholdOn, angleThresholdOff, minimumTimeUp);
};

struct BallHandlingFeedForwardConfig
{
    bool enabledWithoutBall = false;
    bool enabledWithBall = true;
    float factorX = 0.0;
    float factorY = 0.0;
    float factorRz = 0.0;

    SERIALIZE_DATA(enabledWithoutBall, enabledWithBall, factorX, factorY, factorRz);
};

struct BallHandlingExtraPullConfig
{
    // option to generate extra pull force on a lifted ballhandler arm, when the other one is down
    // which can help to bring the ball in
    bool enabledWithoutBall = true;
    bool enabledWithBall = false;
    float setpointVelocity = 300;

    SERIALIZE_DATA(enabledWithoutBall, enabledWithBall, setpointVelocity);
};

struct ConfigBallHandling
{
    // note: angles are fractions, where zero is down (no ball possession) and 1 is up (highest limit)
    // conversion to sensor values is done via arm calibration
    float                         angleSetpoint = 0.8; // control setpoint angle
    float                         armLiftedAngleThreshold = 0.4; // when is arm considered 'lifted'
    BallPossessionConfig          ballPossession;
    BallHandlingFeedForwardConfig feedForward;
    BallHandlingExtraPullConfig   extraPull;
    std::vector<BallHandlingRobotConfig> calibration;

    SERIALIZE_DATA(angleSetpoint, armLiftedAngleThreshold, ballPossession, feedForward, extraPull, calibration);
};

#endif

