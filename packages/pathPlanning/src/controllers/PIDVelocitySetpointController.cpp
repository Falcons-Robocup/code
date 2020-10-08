 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * PIDVelocitySetpointController.cpp
 *
 *  Created on: October, 2019
 *      Author: Jan Feitsma
 */

#include "int/VelocitySetpointControllers.hpp"


bool PIDVelocitySetpointController::calculate(PathPlanningVelocityControlData &data, Velocity2D &resultVelocity)
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
    Position2D delta = data.deltaPosition;
    resultVelocity.x = _controllerX.calculate(delta.x);
    resultVelocity.y = _controllerY.calculate(delta.y);
    resultVelocity.phi = _controllerRz.calculate(delta.phi);

    // store all PID terms for diagnostics / plotting
    data.pidState.x.proportional = _controllerX.terms.proportional;
    data.pidState.x.integral = _controllerX.terms.integral;
    data.pidState.x.derivative = _controllerX.terms.derivative;
    data.pidState.y.proportional = _controllerY.terms.proportional;
    data.pidState.y.integral = _controllerY.terms.integral;
    data.pidState.y.derivative = _controllerY.terms.derivative;
    data.pidState.Rz.proportional = _controllerRz.terms.proportional;
    data.pidState.Rz.integral = _controllerRz.terms.integral;
    data.pidState.Rz.derivative = _controllerRz.terms.derivative;

    // no failure mode defined
    return true;
}

