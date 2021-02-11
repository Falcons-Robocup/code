// Copyright 2015-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cShootPlanner.hpp
 *
 *  Created on: Jun 21, 2015
 *      Author: Thomas Kerkhof
 */

#ifndef CSHOOTPLANNER_HPP_
#define CSHOOTPLANNER_HPP_

#include "falconsCommon.hpp"
#include "shootTypeEnum.hpp"
#include "AbstractInterpolator.hpp" // from package 'filters'

#include "int/adapters/cRTDBOutputAdapter.hpp"
#include "int/cShotSolver.hpp"

typedef struct
{
    bool alwaysLobshot;
    bool alwaysStraightshot;
    double shotStrengthScaling;
    double shotAngleScaling;
    double lobAngleScaling;
    double passScaling;
    double minLobDistance;
    double maxLobDistance;
} shootPlanningParams_t;

class cShootPlanner
{
    public:
        cShootPlanner();
        ~cShootPlanner();

        void prepareForShot(float distance, float z, shootTypeEnum shootType);
        void executeShot();
        void executePass(float distance);

    private:
        
        // models
        bool canPrepare();
        void calculatePassPower(float distance);
        void calculateLob(float distance, float z);
        void calculateStraightShot(float distance, float z);

        T_CONFIG_SHOOTPLANNING getConfig();

        AbstractInterpolator *_passInterpolator;
        cShotSolver *_shotSolver;

        cRTDBOutputAdapter _rtdbOutputAdapter;
        float _kickerStrength;
        float _kickerAngle;
        double _lastSetHeightTimestamp;
};

#endif /* CSHOOTPLANNER_HPP_ */
