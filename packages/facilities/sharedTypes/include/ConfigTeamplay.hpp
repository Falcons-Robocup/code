// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef CONFIGTEAMPLAY_HPP_
#define CONFIGTEAMPLAY_HPP_

#include "RtDB2.h" // required for serialization

#include <map>
#include <string>


struct limits
{
    double minimum;
    double maximum;

    SERIALIZE_DATA(minimum, maximum);
};


struct heightmapConfig
{
    std::map <std::string, std::map <std::string, float> > factors;

    SERIALIZE_DATA(factors);
};


struct interceptBallConfig
{
    double captureRadius;
    double minimumSpeed;
    bool activeIntercept;

    SERIALIZE_DATA(captureRadius, minimumSpeed, activeIntercept);
};


struct rulesConfig
{
    int setpieceExecuteTimeout;
    int penaltyExecuteTimeout;
    double minKickDistanceKicked;
    double minPenaltyDistanceKicked;
    double minOwnKickoffDistanceKicked;

    SERIALIZE_DATA(setpieceExecuteTimeout, penaltyExecuteTimeout, minKickDistanceKicked, minPenaltyDistanceKicked, minOwnKickoffDistanceKicked);
};


struct shootingConfig {
    double shootPathWidth;
    double straightShotThreshold;
    double aimForCornerThreshold;
    limits angleToGoal;
    limits distanceToGoal;

    SERIALIZE_DATA(shootPathWidth, straightShotThreshold, aimForCornerThreshold, angleToGoal, distanceToGoal);
};


struct strategyConfig
{
    bool defendingStrategy;

    SERIALIZE_DATA(defendingStrategy);
};


struct ConfigTeamplay
{
    heightmapConfig heightmaps;
    interceptBallConfig interceptBall;
    rulesConfig rules;
    shootingConfig shooting;
    strategyConfig strategy;

    SERIALIZE_DATA(heightmaps, interceptBall, rules, shooting, strategy);
};


#endif
