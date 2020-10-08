 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * CalculateVelocity.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#include "int/PathPlanningAlgorithms.hpp"
#include "cDiagnostics.hpp"


CalculateVelocity::CalculateVelocity(boost::function<AbstractVelocitySetpointController *(void)> callback)
{
    _callback = callback;
}

void CalculateVelocity::execute(PathPlanningData &data)
{
    TRACE_FUNCTION("");

    // pathPlanning library should ensure some controller is configured
    // we trigger this just-in-time operation via a callback, which makes it possible
    // for instance to overrule runtime whichever controller with Tokyo drift from associated algorithm
    auto controller = _callback();
    assert(controller != NULL);

    // prepare input struct for velocity controller, based on configuration to operate on FCS or RCS
    PathPlanningVelocityControlData vcData;
    vcData.deltaPosition = data.deltaPositionFcs;
    vcData.currentVelocity = data.currentVelocityFcs;
    vcData.previousVelocitySetpoint = data.previousVelocityRcs;
    if (data.vcConfig.coordinateSystem == CoordinateSystemEnum::RCS)
    {
        vcData.deltaPosition = data.deltaPositionRcs;
        vcData.currentVelocity = vcData.currentVelocity.transform_fcs2rcs(data.currentPositionFcs);
        TRACE("controlling velocity in RCS");
    }
    else
    {
        vcData.previousVelocitySetpoint = vcData.previousVelocitySetpoint.transform_rcs2fcs(data.currentPositionFcs);
        TRACE("controlling velocity in FCS");
    }
    vcData.dt = data.dt;
    vcData.currentLimits = data.currentLimits;
    TRACE("delta%s current%s", vcData.deltaPosition.tostr(), vcData.currentVelocity.tostr());

    // call, check result, store output
    Velocity2D resultVelocity;
    bool success = controller->calculate(vcData, resultVelocity);
    if (!success)
    {
        resultVelocity = Velocity2D(0.0, 0.0, 0.0);
        TRACE_WARNING("velocity setpoint calculation failed");
    }

    // PID-specific analysis
    data.pidState = vcData.pidState;

    // if needed, transform from FCS to RCS
    if (data.vcConfig.coordinateSystem == CoordinateSystemEnum::RCS)
    {
        // already in the correct coordinate system
        data.resultVelocityRcs = resultVelocity;
    }
    else
    {
        // need a transformation
        data.resultVelocityRcs = resultVelocity.transform_fcs2rcs(data.currentPositionFcs);
    }

    TRACE("vx=%8.4f vy=%8.4f vphi=%8.4f (RCS) success=%d", data.resultVelocityRcs.x, data.resultVelocityRcs.y, data.resultVelocityRcs.phi, success);
}

