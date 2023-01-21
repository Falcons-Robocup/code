// Copyright 2016-2021 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * configuration.hpp
 *
 *  Created on: Sep 27, 2016
 *      Author: Coen Tempelaars
 */

#ifndef CONFIGURATION_HPP_
#define CONFIGURATION_HPP_

#include <map>
#include <vector>

#include "ConfigTeamplay.hpp" // sharedTypes
#include "ext/heightmapNames.hpp"
#include "int/types/heightmapEnumTypes.hpp"


namespace teamplay
{

class configuration {
public:
    configuration();
    virtual ~configuration();

    virtual void update(const ConfigTeamplay&);

    virtual float getSetPieceExecuteTimeoutSeconds() const;
    virtual float getPenaltyExecuteTimeoutSeconds() const;
    virtual float getMinKickDistanceKickedMeters() const;
    virtual float getMinPenaltyDistanceKickedMeters() const;
    virtual float getMinOwnKickoffDistanceKickedMeters() const;
    virtual float getShootPathWidth() const;
    virtual float getStraightShotThreshold() const;
    virtual float getAimForCornerThreshold() const;
    virtual float getMinimumAngleToGoal() const;
    virtual float getMaximumAngleToGoal() const;
    virtual float getMinimumDistanceToGoal() const;
    virtual float getMaximumDistanceToGoal() const;
    virtual float getInterceptBallCaptureRadius() const;
    virtual float getInterceptBallMinimumSpeed() const;
    virtual bool isActiveInterceptEnabled() const;
    virtual bool getDefendingStrategy() const;
    virtual float getHeightmapFactor (const CompositeHeightmapName& compositeHeightmap, const heightmapEnum& heightmap) const;

private:
    ConfigTeamplay _config;
};


} /* namespace teamplay */

#endif /* CONFIGURATION_HPP_ */
