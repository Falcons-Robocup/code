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

#include "int/algorithms/cYPID.hpp"

/* Overwrite functionality of cAbstractPathPlanning update function */
void cYPID::execute()
{
    TRACE(">");
    // Get current position from cPathPlanningData
    Position2D currPos;
    _main->_ppData->getPosition(currPos);

    // Get current velocity from cPathPlanningData
    Velocity2D currVel;
    _main->_ppData->getVelocity(currVel);

    // Get PID parameters (reconfig) from cPathPlanningData
    pp_pid_params_struct_t pidParams;
    _main->_ppData->getPIDParams(pidParams);

    // Get target position from cPathPlanningData
    Position2D targetPosition;
    _main->_ppData->getTarget(targetPosition);

    Position2D deltaPosition;
    _main->_ppData->getDeltaPosition(deltaPosition);

    /* Declare local variables */
    double yerror = 0.0;

    double yerrorsum = 0.0;

    double yerrorD = 0.0;

    Velocity2D newVel;

    /* Calculate PID function */
    yerror = deltaPosition.y;

    /* integral = integral + (error * dt) */
    yerrorsum = yerrorsum + yerror * _dt;

    /* derivative = (error - previous_error) / dt */
    yerrorD = (yerror - _prev_vel.y) / _dt;


    newVel.y = pidParams.Y_P * yerror
             + pidParams.Y_I * yerrorsum
             + pidParams.Y_D * yerrorD;

    /* Save errors for D-action later */
    _prev_vel.y = yerror;

    _ppData.vel.y = newVel.y;
    TRACE("<");
}

