// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * PIDVelocitySetpointController.cpp
 *
 *  Created on: October, 2019
 *      Author: Jan Feitsma
 */

#include "int/VelocitySetpointControllers.hpp"


bool PIDVelocitySetpointController::calculate(VelocityControlData &data, Velocity2D &resultVelocity)
{
    TRACE_FUNCTION("");

    // 3 degrees of freedom are treated independently
    // in RCS or FCS, depending on configuration

    // (re-)configure each controller
    auto pidParams = _config;
    _controllerX.params.dt = data.dt;
    _controllerX.params.P = pidParams.XY_P;
    _controllerX.params.I = pidParams.XY_I;
    _controllerX.params.D = pidParams.XY_D;
    _controllerX.params.maxI = pidParams.maxI_XY;
    _controllerX.params.fadeI = pidParams.fadeI_XY;
    _controllerY.params.dt = data.dt;
    _controllerY.params.P = pidParams.XY_P;
    _controllerY.params.I = pidParams.XY_I;
    _controllerY.params.D = pidParams.XY_D;
    _controllerY.params.maxI = pidParams.maxI_XY;
    _controllerY.params.fadeI = pidParams.fadeI_XY;
    _controllerRz.params.dt = data.dt;
    _controllerRz.params.P = pidParams.RZ_P;
    _controllerRz.params.I = pidParams.RZ_I;
    _controllerRz.params.D = pidParams.RZ_D;
    _controllerRz.params.maxI = pidParams.maxI_Rz;
    _controllerRz.params.fadeI = pidParams.fadeI_Rz;

    // calculate
    Position2D delta; // in RCS or FCS depending on configuration

    if (data.robotPosVelMoveType == robotPosVelEnum::POSVEL || data.robotPosVelMoveType == robotPosVelEnum::POS_ONLY)
    {
        if (data.vcSetpointConfig.coordinateSystem == CoordinateSystemEnum::RCS)
        {
            delta = data.deltaPositionRcs;
            TRACE("controlling velocity in RCS");
        }
        else
        {
            delta = data.deltaPositionFcs;
            TRACE("controlling velocity in FCS");
        }

        resultVelocity.x = _controllerX.calculate(delta.x);
        resultVelocity.y = _controllerY.calculate(delta.y);
        resultVelocity.phi = _controllerRz.calculate(delta.phi);
    }
    else if (data.robotPosVelMoveType == robotPosVelEnum::VEL_ONLY)
    {
        // VEL_ONLY + PID = pass-through velocity.
        // limiters will take care of profile
        resultVelocity = Velocity2D(data.target.vel.x, data.target.vel.y, data.target.vel.Rz);
    }

    // store all PID terms for diagnostics / plotting
    //data.pidState.x.proportional = _controllerX.terms.proportional;
    //data.pidState.x.integral = _controllerX.terms.integral;
    //data.pidState.x.derivative = _controllerX.terms.derivative;
    //data.pidState.y.proportional = _controllerY.terms.proportional;
    //data.pidState.y.integral = _controllerY.terms.integral;
    //data.pidState.y.derivative = _controllerY.terms.derivative;
    //data.pidState.Rz.proportional = _controllerRz.terms.proportional;
    //data.pidState.Rz.integral = _controllerRz.terms.integral;
    //data.pidState.Rz.derivative = _controllerRz.terms.derivative;

    // no failure mode defined
    return true;
}

