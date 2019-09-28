 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPIDPathPlanning.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Tim Kouters
 */

#include "int/algorithms/cXYBrake.hpp"

/* Overwrite functionality of cAbstractPathPlanning update function */
void cXYBrake::execute()
{
    TRACE(">");

    // Get current position from cPathPlanningData
    Position2D currPos;
    _main->_ppData->getPosition(currPos);

    // Get current velocity from cPathPlanningData
    Velocity2D currVel;
    _main->_ppData->getVelocity(currVel);

    // Get Brae parameters (reconfig) from cPathPlanningData
    pp_brake_params_struct_t brakeParams;
    _main->_ppData->getBrakeParams(brakeParams);

    // Get target position from cPathPlanningData
    Position2D targetPosition;
    _main->_ppData->getTarget(targetPosition);

    // Get limits (reconfig) from cPathPlanningData
    pp_limiters_struct_t limits;
    _main->_ppData->getLimits(limits);

    // Initialize on current velocity
    Velocity2D newVel = currVel;

    // Braking distance = curVel / 2 * A_dec
    // When braking, Vnew = vCur * gamma
    double b = fabs(currVel.size() / (2 * brakeParams.deceleration));

    // Print: Distance to target.
    // current velocity
    // braking distance
    TRACE("Distance to target: %12.9f; CurVel: %12.9f; BrakingDist: %12.9f", (currPos - targetPosition).size(), currVel.size(), b);

    // If braking distance <= distance to target
    if ( b <= fabs((currPos - targetPosition).size()) )
    {
        // Otherwise set velocity to error, normalize, and multiply with maxVel
        newVel.x = (targetPosition - currPos).x;
        newVel.y = (targetPosition - currPos).y;

        if (newVel.size() > 0.0)
        {
            newVel.x = (newVel.xy().normalize() * limits.maxVelXY).x;
            newVel.y = (newVel.xy().normalize() * limits.maxVelXY).y;
        }

        TRACE("Setting velocity: (%12.9f, %12.9f)", newVel.x, newVel.y);
    }
    else
    {
        // Start braking
        TRACE("Braking!");
        newVel.x = (currVel.xy() * brakeParams.gamma).x;
        newVel.y = (currVel.xy() * brakeParams.gamma).y;
    }

    _ppData.vel.x = newVel.x;
    _ppData.vel.y = newVel.y;

    TRACE("<");
}

