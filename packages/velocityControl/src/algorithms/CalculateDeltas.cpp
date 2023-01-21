// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * CalculateDeltas.cpp
 *
 *  Created on: October, 2019
 *      Author: Jan Feitsma
 */

#include "int/VelocityControlAlgorithms.hpp"
#include "int/facilities/vcgeometry.hpp"


void CalculateDeltas::execute(VelocityControlData &data)
{
    TRACE_FUNCTION("");

    // get uncorrected data
    data.targetPositionFcs = Position2D(data.target.pos.x, data.target.pos.y, data.target.pos.Rz);
    data.targetVelocityFcs = Velocity2D(data.target.vel.x, data.target.vel.y, data.target.vel.Rz);
    data.currentPositionFcs = Position2D(data.robot.position.x, data.robot.position.y, data.robot.position.Rz);
    data.currentVelocityFcs = Velocity2D(data.robot.velocity.x, data.robot.velocity.y, data.robot.velocity.Rz);


    // UnshiftBallPos -> CalculateDeltas -> Deadzone expects:
    //      targetPositionFcs / currentPositionFcs
    //      ->
    //      deltaPositionRcs

    //float w_pos = data.currentMotionTypeConfig.setPointGenerator.weightFactorClosedLoopPos;
    //data.weightedCurrentPositionFcs = data.currentPositionFcs * w_pos + data.previousPositionSetpointFcs * (1.0-w_pos);

    //float w_vel = data.currentMotionTypeConfig.setPointGenerator.weightFactorClosedLoopVel;
    //data.weightedCurrentVelocityFcs = data.currentVelocityFcs * w_vel + data.previousVelocitySetpointFcs * (1.0-w_vel);

    //// delta position FCS
    //data.deltaPositionFcs = data.targetPositionFcs - data.weightedCurrentPositionFcs;
    //data.deltaPositionFcs.phi = project_angle_mpi_pi(data.deltaPositionFcs.phi);
    //// TODO: simplify these operations, see #14
    //TRACE("deltaPositionFcs%s", data.deltaPositionFcs.tostr());

    //// delta position RCS
    //data.deltaPositionRcs = Position2D(data.targetPositionFcs).transform_fcs2rcs(data.weightedCurrentPositionFcs);
    //data.deltaPositionRcs.phi = data.deltaPositionFcs.phi;
    //TRACE("deltaPositionRcs%s", data.deltaPositionRcs.tostr());

    //// delta velocity RCS
    //Velocity2D currentVelocityRcs = data.weightedCurrentVelocityFcs;
    //data.weightedCurrentVelocityRcs = currentVelocityRcs.transform_fcs2rcs(data.weightedCurrentPositionFcs);
    //Velocity2D targetVelocityRcs = Velocity2D(data.target.vel.x, data.target.vel.y, data.target.vel.Rz);
    //data.deltaVelocityRcs = targetVelocityRcs - currentVelocityRcs;
    //TRACE("currentVelocityRcs=%s, targetVelocityRcs=%s, deltaVelocityRcs=%s", currentVelocityRcs.tostr(), targetVelocityRcs.tostr(), data.deltaVelocityRcs.tostr());
}

