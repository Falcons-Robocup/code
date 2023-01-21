// Copyright 2019-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * PathPlanningData.hpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#ifndef PATHPLANNINGDATA_HPP_
#define PATHPLANNINGDATA_HPP_

// system
#include <vector>

// other Falcons packages
#include "falconsCommon.hpp" // TODO disentangle (move Position2D etc. to geometry package? #14)

// sharedTypes
#include "actionResult.hpp"
#include "ConfigPathPlanning.hpp"
#include "ConfigExecution.hpp"
#include "diagPathPlanning.hpp"
#include "wayPoint.hpp"
#include "forbiddenArea.hpp"
#include "motionSetpoint.hpp"
#include "obstacleResult.hpp"
#include "ballResult.hpp"
#include "robotState.hpp"
#include "robotVelocity.hpp"


struct PathPlanningData
{
    // inputs
    ConfigPathPlanning          configPP;
    ConfigExecution             configEx;
    wayPoint                    target;
    rtime                       timestamp;
    std::vector<forbiddenArea>  forbiddenAreas;
    robotState                  robot;
    std::vector<ballResult>     balls;
    std::vector<robotState>     teamMembers;
    std::vector<obstacleResult> obstacles; // only the ones from worldModel, see calculatedObstacles below

    // calculation results
    std::vector<wayPoint>       path;
    actionResultTypeEnum        resultStatus;
    std::vector<forbiddenArea>  calculatedForbiddenAreas; // = input + obstacle paths
    std::vector<obstacleResult> calculatedObstacles;
    Position2D                  targetPositionFcs; // might be corrected with ball possession offset
    Position2D                  currentPositionFcs; // might be corrected with ball possession offset
    Position2D                  deltaPositionFcs; // delta of current position w.r.t. subtarget (=first waypoint)
    Position2D                  deltaPositionRcs;

    // internal data
    float                       dt;
    bool                        done = false;
    motionTypeEnum              motionType = motionTypeEnum::INVALID;
    bool                        stop = false;
    rtime                       previousTimestamp;

    // functions
    void reset();
    void traceInputs();
    void traceOutputs();
    Position2D getSubTarget() const;
    void insertSubTarget(Position2D const &pos, Velocity2D const &vel = Velocity2D(0,0,0));
    void addForbiddenAreas(std::vector<forbiddenArea> const &newForbiddenAreas);
    void addForbiddenArea(forbiddenArea const &newForbiddenArea);
};

#endif

