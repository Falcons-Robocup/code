 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * CalculateAccelerating.cpp
 *
 *  Created on: December, 2019
 *      Author: Jan Feitsma
 */

#include "int/PathPlanningAlgorithms.hpp"


void CalculateAccelerating::execute(PathPlanningData &data)
{
    TRACE_FUNCTION("");

    // calculate current velocity in RCS
    Velocity2D c = data.currentVelocityFcs;
    c = c.transform_fcs2rcs(data.currentPositionFcs);

    // calculate acceleration based on setpoints (real acceleration typically lags behind a bit)
    auto a = (data.resultVelocityRcs - data.previousVelocityRcs) / data.dt;

    // take inproduct of acceleration with current velocity to determine if robot is accelerating or not
    data.isAccelerating[0] = (a.x * c.x >= data.currentLimits.accThresholdX);
    data.isAccelerating[1] = (a.y * c.y >= data.currentLimits.accThresholdY);
    data.isAccelerating[2] = (a.phi * c.phi >= data.currentLimits.accThresholdRz);

    TRACE("isAccX=%d isAccY=%d isAccRz=%d", data.isAccelerating[0], data.isAccelerating[1], data.isAccelerating[2]);
}

