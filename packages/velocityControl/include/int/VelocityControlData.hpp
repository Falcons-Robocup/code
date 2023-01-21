// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * VelocityControlData.hpp
 *
 *  Created on: Oct, 2020
 *      Author: Erik Kouters
 */

#ifndef VELOCITYCONTROLDATA_HPP_
#define VELOCITYCONTROLDATA_HPP_

// system
#include <vector>

// other Falcons packages
#include "falconsCommon.hpp" // TODO disentangle (move Position2D etc. to geometry package? #14)

#include "cVelocityTransformClient.hpp"

// sharedTypes
#include "ConfigVelocityControl.hpp"
#include "ConfigPathPlanning.hpp"
#include "ConfigExecution.hpp"
#include "diagVelocityControl.hpp"
#include "wayPoint.hpp"
#include "motionSetpoint.hpp"
#include "obstacleResult.hpp"
#include "ballResult.hpp"
#include "robotState.hpp"
#include "robotVelocity.hpp"
#include "robotPosVel.hpp"


struct VelocityControlData
{
    // inputs
    cVelocityTransformClient    vtClient;
    ConfigVelocityControl       vcConfig;
    ConfigPathPlanning          ppConfig;
    ConfigExecution             exConfig;
    wayPoint                    target;
    rtime                       timestamp;
    robotState                  robot;
    std::vector<ballResult>     balls;
    std::vector<robotState>     teamMembers;
    motionTypeEnum              motionType = motionTypeEnum::INVALID;
    robotPosVelEnum             robotPosVelMoveType = robotPosVelEnum::INVALID;

    // calculation results
    VelocitySetpointControllerConfig vcSetpointConfig;
    Position2D                  currentPositionFcs; // might be corrected with ball possession offset
    Velocity2D                  currentVelocityFcs; // might be corrected with ball possession offset
    Position2D                  targetPositionFcs; // might be corrected with ball possession offset
    Velocity2D                  targetVelocityFcs;
    Position2D                  deltaPositionFcs; // delta of current position w.r.t. subtarget (=first waypoint)
    Position2D                  deltaPositionRcs;
    Velocity2D                  deltaVelocityRcs;
    bool                        shortStroke;
    Velocity2D                  resultVelocityRcs;
    pose                        accelerationRcs; // TODO #14 make consistent
    bool                        isAccelerating[3];
    bool                        accelerationClipping[3];
    bool                        deadzone[3];

    // internal data
    float                       dt;
    bool                        done = false;
    MotionTypeConfig            currentMotionTypeConfig;
    Position2D                  previousPositionFcs;
    Position2D                  previousPositionSetpointFcs;
    Velocity2D                  previousVelocitySetpointFcs;
    Velocity2D                  previousVelocityRcs; // store from previous iteration, for acceleration limiter
    Velocity2D                  previousVelocityFcs; // store from previous iteration, for acceleration limiter
    rtime                       previousTimestamp;

    // SPG diag data
    pose spgCurrentPosition;
    pose spgCurrentVelocity;
    pose spgMaxVelocity;
    pose spgMaxAcceleration;
    pose spgTargetPosition;
    pose spgTargetVelocity;
    pose spgNewPosition;
    pose spgNewVelocity;

    // functions
    void reset();
    void traceInputs();
    void traceOutputs();
    void configureLimits();
};

#endif

