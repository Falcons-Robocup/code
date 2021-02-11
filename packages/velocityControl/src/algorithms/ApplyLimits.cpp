// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ApplyLimits.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#include "int/VelocityControlAlgorithms.hpp"
#include "int/facilities/clipping.hpp"


void ApplyLimits::execute(VelocityControlData &data)
{
    TRACE_FUNCTION("");

    // trace limits
    TRACE("using limits: %s", tostr(data.currentMotionTypeConfig).c_str());

    // get inputs
    Velocity2D unclippedVelocitySetpoint = data.resultVelocityRcs;
    Velocity2D previousVelocitySetpoint = data.previousVelocityRcs;
    float dt = data.dt;
    float maxSpeedX = data.currentMotionTypeConfig.limits.maxVelX;
    bool drivingBackward = unclippedVelocitySetpoint.y < 0.0;
    float maxSpeedY = (drivingBackward ? data.currentMotionTypeConfig.limits.maxVelYbackward : data.currentMotionTypeConfig.limits.maxVelYforward);
    float maxSpeedRz = data.currentMotionTypeConfig.limits.maxVelRz;
    Velocity2D currentVelocity = Velocity2D(data.robot.velocity.x, data.robot.velocity.y, data.robot.velocity.Rz);
    currentVelocity.transform_fcs2rcs(Position2D(data.robot.position.x, data.robot.position.y, data.robot.position.Rz));

    // initialize result
    Velocity2D newVelocitySetpoint = unclippedVelocitySetpoint;

    // XY velocity and acceleration
    if (data.isAccelerating[0])
    {
        if (gclip(newVelocitySetpoint.x, previousVelocitySetpoint.x, data.currentMotionTypeConfig.limits.maxAccX * dt, "accX"))
        {
            data.accelerationClipping[0] = true;
        }
    }
    else
    {
        if (gclip(newVelocitySetpoint.x, previousVelocitySetpoint.x, data.currentMotionTypeConfig.limits.maxDecX * dt, "decX"))
        {
            data.accelerationClipping[0] = true;
        }
    }
    (void)gclip(newVelocitySetpoint.x, 0.0, maxSpeedX, "velX");
    if (data.isAccelerating[1])
    {
        float maxAccY = data.currentMotionTypeConfig.limits.maxAccYforward;
        if (drivingBackward)
        {
            maxAccY = data.currentMotionTypeConfig.limits.maxAccYbackward;
        }
        if (gclip(newVelocitySetpoint.y, previousVelocitySetpoint.y, maxAccY * dt, "accY"))
        {
            data.accelerationClipping[1] = true;
        }
    }
    else
    {
        if (gclip(newVelocitySetpoint.y, previousVelocitySetpoint.y, data.currentMotionTypeConfig.limits.maxDecY * dt, "decY"))
        {
            data.accelerationClipping[1] = true;
        }
    }
    (void)gclip(newVelocitySetpoint.y, 0.0, maxSpeedY, "velY");

    // Rz velocity and acceleration
    if (data.isAccelerating[2])
    {
        if (gclip(newVelocitySetpoint.phi, previousVelocitySetpoint.phi, data.currentMotionTypeConfig.limits.maxAccRz * dt, "accRz"))
        {
            data.accelerationClipping[2] = true;
        }
    }
    else
    {
        if (gclip(newVelocitySetpoint.phi, previousVelocitySetpoint.phi, data.currentMotionTypeConfig.limits.maxDecRz * dt, "decRz"))
        {
            data.accelerationClipping[2] = true;
        }
    }
    (void)gclip(newVelocitySetpoint.phi, 0.0, maxSpeedRz, "velRz");

    // store
    // NOTE: in case of ball possession, this velocity- and acceleration result applies to ball, not robot!
    // TODO: this might be getting a bit confusing, better to split? for example, kstplot will now show inconsistent diagnostics ...
    data.resultVelocityRcs = newVelocitySetpoint;
    auto a = (data.resultVelocityRcs - data.previousVelocityRcs) / data.dt; // TODO #14
    data.accelerationRcs = pose(a.x, a.y, a.phi);
    data.previousVelocityRcs = data.resultVelocityRcs;
    data.previousTimestamp = data.timestamp;
}

