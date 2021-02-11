// Copyright 2016-2020 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * configuration.cpp
 *
 *  Created on: Sep 27, 2016
 *      Author: Coen Tempelaars
 */

#include "int/types/configuration.hpp"
#include "ext/heightmapNames.hpp"
#include "tracing.hpp"

using namespace teamplay;

configuration::configuration()
{
}

configuration::~configuration()
{
}

void configuration::update (const configTeamplay& c)
{
    _config = c;
}

float configuration::getSetPieceExecuteTimeoutSeconds() const
{
    return _config.rules.setpieceExecuteTimeout;
}

float configuration::getPenaltyExecuteTimeoutSeconds() const
{
    return _config.rules.penaltyExecuteTimeout;
}

float configuration::getMinKickDistanceKickedMeters() const
{
    return _config.rules.minKickDistanceKicked;
}

float configuration::getMinPenaltyDistanceKickedMeters() const
{
    return _config.rules.minPenaltyDistanceKicked;
}

float configuration::getMinOwnKickoffDistanceKickedMeters() const
{
    return _config.rules.minOwnKickoffDistanceKicked;
}

float configuration::getShootPathWidth() const
{
    return _config.shooting.shootPathWidth;
}

float configuration::getStraightShotThreshold() const
{
    return _config.shooting.straightShotThreshold;
}

float configuration::getAimForCornerThreshold() const
{
    return _config.shooting.aimForCornerThreshold;
}

float configuration::getMinimumAngleToGoal() const
{
    return _config.shooting.angleToGoal.minimum;
}

float configuration::getMaximumAngleToGoal() const
{
    return _config.shooting.angleToGoal.maximum;
}

float configuration::getMinimumDistanceToGoal() const
{
    return _config.shooting.distanceToGoal.minimum;
}

float configuration::getMaximumDistanceToGoal() const
{
    return _config.shooting.distanceToGoal.maximum;
}

float configuration::getInterceptBallCaptureRadius() const // TODO cleanup? (moved to motionPlanning?)
{
    return _config.interceptBall.captureRadius;
}

float configuration::getInterceptBallMinimumSpeed() const // TODO cleanup? (moved to motionPlanning?)
{
    return _config.interceptBall.minimumSpeed;
}

bool configuration::isActiveInterceptEnabled() const
{
    return _config.interceptBall.activeIntercept;
}

bool configuration::getDefendingStrategy() const
{
    return _config.strategy.defendingStrategy;
}

float configuration::getHeightmapFactor (const CompositeHeightmapName& compositeHeightmap, const heightmapEnum& heightmap) const
{
    auto factors = _config.heightmaps.factors.find(enum2str(compositeHeightmap));
    if (factors != _config.heightmaps.factors.end())
    {
        auto factor = factors->second.find(enum2str(heightmap));
        if (factor != factors->second.end())
        {
            return factor->second;
        }
    }

    TRACE("Factor not found. Returning zero.");
    return 0.0;
}

