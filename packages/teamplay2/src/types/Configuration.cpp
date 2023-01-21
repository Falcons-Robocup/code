// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Configuration.cpp
 *
 *  Created on: Sep 27, 2016
 *      Author: Coen Tempelaars
 */

#include "int/types/Configuration.hpp"
#include "HeightmapNames.hpp"
#include "tracing.hpp"

using namespace teamplay;

Configuration::Configuration()
    : _configAdapterTP(CONFIG_TEAMPLAY),
      _configAdapterEx(CONFIG_EXECUTION)
{
    std::string configFile = determineConfig("teamplay");
    _configAdapterTP.setConfigUpdateCallback(boost::bind(&Configuration::update, this));
    _configAdapterTP.loadYAML(configFile, false);

    configFile = determineConfig("execution");
    _configAdapterEx.setConfigUpdateCallback(boost::bind(&Configuration::update, this));
    _configAdapterEx.loadYAML(configFile, false);
}

Configuration::~Configuration()
{
}

void Configuration::update ()
{
    _configAdapterTP.get(_configTP);
    _configAdapterEx.get(_configEx);
}

float Configuration::getSetPieceExecuteTimeoutSeconds() const
{
    return _configTP.rules.setpieceExecuteTimeout;
}

float Configuration::getPenaltyExecuteTimeoutSeconds() const
{
    return _configTP.rules.penaltyExecuteTimeout;
}

float Configuration::getMinKickDistanceKickedMeters() const
{
    return _configTP.rules.minKickDistanceKicked;
}

float Configuration::getMinPenaltyDistanceKickedMeters() const
{
    return _configTP.rules.minPenaltyDistanceKicked;
}

float Configuration::getMinOwnKickoffDistanceKickedMeters() const
{
    return _configTP.rules.minOwnKickoffDistanceKicked;
}

float Configuration::getShootPathWidth() const
{
    return _configTP.shooting.shootPathWidth;
}

float Configuration::getStraightShotThreshold() const
{
    return _configTP.shooting.straightShotThreshold;
}

float Configuration::getAimForCornerThreshold() const
{
    return _configTP.shooting.aimForCornerThreshold;
}

float Configuration::getMinimumAngleToGoal() const
{
    return _configTP.shooting.angleToGoal.minimum;
}

float Configuration::getMaximumAngleToGoal() const
{
    return _configTP.shooting.angleToGoal.maximum;
}

float Configuration::getMinimumDistanceToGoal() const
{
    return _configTP.shooting.distanceToGoal.minimum;
}

float Configuration::getMaximumDistanceToGoal() const
{
    return _configTP.shooting.distanceToGoal.maximum;
}

float Configuration::getInterceptBallCaptureRadius() const // TODO cleanup? (moved to motionPlanning?)
{
    return _configTP.interceptBall.captureRadius;
}

float Configuration::getInterceptBallMinimumSpeed() const // TODO cleanup? (moved to motionPlanning?)
{
    return _configTP.interceptBall.minimumSpeed;
}

bool Configuration::isActiveInterceptEnabled() const
{
    return _configTP.interceptBall.activeIntercept;
}

bool Configuration::getDefendingStrategy() const
{
    return _configTP.strategy.defendingStrategy;
}

float Configuration::getHeightmapFactor (const CompositeHeightmapName& compositeHeightmap, const HeightmapEnum& heightmap) const
{
    auto factors = _configTP.heightmaps.factors.find(enum2str(compositeHeightmap));
    if (factors != _configTP.heightmaps.factors.end())
    {
        auto factor = factors->second.find(enum2str(heightmap));
        if (factor != factors->second.end())
        {
            return factor->second;
        }
    }

    std::runtime_error("Factor not found. Returning zero.");
    return 0.0;
}

float Configuration::getNominalFrequency() const
{
    return _configEx.frequency;
}

