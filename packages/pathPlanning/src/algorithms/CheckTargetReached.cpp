 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * CheckTargetReached.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#include "int/PathPlanningAlgorithms.hpp"


void CheckTargetReached::execute(PathPlanningData &data)
{
    TRACE_FUNCTION("");
    // get delta - TODO simplify
    Position2D targetPos = data.getSubTarget();
    Position2D robotPos(data.robot.position.x, data.robot.position.y, data.robot.position.Rz);
    Position2D deltaPositionFcs = targetPos - robotPos;
    deltaPositionFcs.phi = project_angle_mpi_pi(deltaPositionFcs.phi);
    // compare with tolerances
    bool xyOk = deltaPositionFcs.xy().size() < data.currentLimits.toleranceXY;
    bool RzOk = fabs(deltaPositionFcs.phi) < data.currentLimits.toleranceRz;
    TRACE("xyOk=%d RzOk=%d", xyOk, RzOk);
    // convergence criterion, especially useful for testing where overshoot can cause premature 'PASSED'
    static int tickCountTargetReached = 0;
    // update data
    if (xyOk && RzOk)
    {
        tickCountTargetReached++;
        if (tickCountTargetReached >= data.config.numExtraSettlingTicks)
        {
            data.resultStatus = actionResultTypeEnum::PASSED;
            data.done = true;
        }
    }
    else
    {
        tickCountTargetReached = 0;
        data.resultStatus = actionResultTypeEnum::RUNNING;
    }
}

