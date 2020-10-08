 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Deadzone.cpp
 *
 *  Created on: January, 2020
 *      Author: Jan Feitsma
 */

#include "int/PathPlanningAlgorithms.hpp"
#include "cDiagnostics.hpp"


void Deadzone::execute(PathPlanningData &data)
{
    TRACE_FUNCTION("");

    // compare with tolerances
    bool xyOk = data.deltaPositionRcs.xy().size() < data.currentLimits.toleranceXY;
    bool RzOk = fabs(data.deltaPositionRcs.phi) < data.currentLimits.toleranceRz;
    TRACE("xyOk=%d RzOk=%d", xyOk, RzOk);

    // "deadzone" overrule: in case of partial convergence, set zero output
    // this is especially useful during tuning, to prevent a tiny Rz move setpoint interfering with XY
    data.deadzone[0] = false;
    data.deadzone[1] = false;
    data.deadzone[2] = false;
    if (data.config.deadzone)
    {
        if (xyOk)
        {
            data.resultVelocityRcs.x = 0.0;
            data.resultVelocityRcs.y = 0.0;
            data.deadzone[0] = true;
            data.deadzone[1] = true;
            TRACE("deadzone XY -> setting vx=0 and vy=0");
        }
        if (RzOk)
        {
            data.resultVelocityRcs.phi = 0.0;
            data.deadzone[2] = true;
            TRACE("deadzone Rz -> setting vphi=0");
        }
        if (!xyOk && !RzOk)
        {
            TRACE("deadzone not applied");
        }
    }
    else
    {
        TRACE("deadzone option disabled");
    }
}

