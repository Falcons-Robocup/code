// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Configuration.hpp
 *
 *  Created on: Sep 27, 2016
 *      Author: Coen Tempelaars
 */

#ifndef CONFIGURATION_HPP_
#define CONFIGURATION_HPP_

#include <map>
#include <vector>

#include "ConfigTeamplay.hpp" // sharedTypes
#include "ConfigExecution.hpp" // sharedTypes
#include "HeightmapNames.hpp" // sharedTypes
#include "int/types/HeightmapEnumTypes.hpp"

#include "FalconsRTDB.hpp"
#include "ConfigRTDBAdapter.hpp"

namespace teamplay
{

class Configuration {
public:
    Configuration();
    virtual ~Configuration();

    virtual void update();

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
    virtual float getHeightmapFactor (const CompositeHeightmapName& compositeHeightmap, const HeightmapEnum& heightmap) const;

    // From Execution
    virtual float getNominalFrequency() const;

private:
    ConfigRTDBAdapter<T_CONFIG_TEAMPLAY> _configAdapterTP;
    ConfigRTDBAdapter<T_CONFIG_EXECUTION> _configAdapterEx;
    T_CONFIG_TEAMPLAY _configTP;
    T_CONFIG_EXECUTION _configEx;
};


} /* namespace teamplay */

#endif /* CONFIGURATION_HPP_ */
