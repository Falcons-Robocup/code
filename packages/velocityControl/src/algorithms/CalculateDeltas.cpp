// Copyright 2020 Erik Kouters (Falcons)
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

    // delta position FCS
    data.deltaPositionFcs = data.targetPositionFcs - data.currentPositionFcs;
    data.deltaPositionFcs.phi = project_angle_mpi_pi(data.deltaPositionFcs.phi);
    // TODO: simplify these operations, see #14
    TRACE("deltaPositionFcs%s", data.deltaPositionFcs.tostr());

    // delta position RCS
    data.deltaPositionRcs = Position2D(data.targetPositionFcs).transform_fcs2rcs(data.currentPositionFcs);
    data.deltaPositionRcs.phi = data.deltaPositionFcs.phi;
    TRACE("deltaPositionRcs%s", data.deltaPositionRcs.tostr());

    // delta velocity RCS
    Velocity2D currentVelocityRcs = data.currentVelocityFcs;
    currentVelocityRcs = currentVelocityRcs.transform_fcs2rcs(data.currentPositionFcs);
    Velocity2D targetVelocityRcs = Velocity2D(data.target.vel.x, data.target.vel.y, data.target.vel.Rz);
    data.deltaVelocityRcs = targetVelocityRcs - currentVelocityRcs;
    TRACE("currentVelocityRcs=%s, targetVelocityRcs=%s, deltaVelocityRcs=%s", currentVelocityRcs.tostr(), targetVelocityRcs.tostr(), data.deltaVelocityRcs.tostr());
}

