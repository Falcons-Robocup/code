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

#include "int/algorithms/cXYPID.hpp"

/* Overwrite functionality of cAbstractPathPlanning update function */
void cXYPID::execute()
{
    TRACE_FUNCTION("");
    //TRACE(">");

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

    //plotdata
    pp_plot_data_struct_t plotdata;
    _main->_ppData->getPlotData(plotdata);

    plotdata.vel_a.x = currVel.x;
    plotdata.vel_a.y = currVel.y;
    plotdata.vel_a.phi = currVel.phi;

    Velocity2D newVel;

    /* Calculate PID function */
    double xerror = _ppData.pos.x - currPos.x;
    double yerror = _ppData.pos.y - currPos.y;

    /* integral = integral + (error * dt) */
    _integral.x = _integral.x + xerror * _dt;
    _integral.y = _integral.y + yerror * _dt;

    if (_integral.x > pidParams.maxI_XY)
    {
        _integral.x = pidParams.maxI_XY;
    }
    else if (_integral.x < -(pidParams.maxI_XY))
    {
        _integral.x = -(pidParams.maxI_XY);
    }

    if (_integral.y > pidParams.maxI_XY)
    {
        _integral.y = pidParams.maxI_XY;
    }
    else if (_integral.y < -(pidParams.maxI_XY))
    {
        _integral.y = -(pidParams.maxI_XY);
    }

    /* derivative = (error - previous_error) / dt */
    double xderivative = (xerror - _prev_vel.x) / _dt;
    double yderivative = (yerror - _prev_vel.y) / _dt;


    newVel.x =    pidParams.XY_P * xerror
                + pidParams.XY_I * _integral.x
                + pidParams.XY_D * xderivative;
    newVel.y =    pidParams.XY_P * yerror
                + pidParams.XY_I * _integral.y
                + pidParams.XY_D * yderivative;

    /* Save errors for D-action later */
    _prev_vel.x = xerror;
    _prev_vel.y = yerror;

//    std::stringstream ss;
//    ss << "PIDPLOT " <<
//            pidParams.XY_P << " " << pidParams.XY_I << " " << pidParams.XY_D << " " <<                                             /* P I D                  */
//            pidParams.XY_P * xerror << " " << pidParams.XY_P * yerror << " " <<          /* Px Py             */
//            pidParams.XY_I * _integral.x << " " << pidParams.XY_I * _integral.y << " " <<  /* Ix Iy             */
//            pidParams.XY_D * xderivative << " " << pidParams.XY_D * yderivative << " " <<        /* Dx Dy             */
//            newVel.x << " " << newVel.y << " " <<                                                             /* vx vy               */
//            currPos.x << " " << currPos.y << " " <<                                                          /* cposx cposy     */
//            targetPosition.x << " " << targetPosition.y << " " <<                                     /* tposx tposy   */
//            currVel.x << " " << currVel.y << std::endl;
//    TRACE(ss.str().c_str());

    _ppData.vel.x = newVel.x;
    _ppData.vel.y = newVel.y;

    plotdata.pos_t = targetPosition;
    plotdata.pos_a = currPos;


    plotdata.x_pid_out = newVel.x;
    plotdata.y_pid_out = newVel.y;
    plotdata.x_pid_p = pidParams.XY_P * xerror;
    plotdata.y_pid_p = pidParams.XY_P * yerror;
    plotdata.x_pid_i = pidParams.XY_I * _integral.x;
    plotdata.y_pid_i = pidParams.XY_I * _integral.y;
    plotdata.x_pid_d = pidParams.XY_D * xderivative;
    plotdata.y_pid_d = pidParams.XY_D * yderivative;

    _main->_ppData->setPlotData(plotdata);

    //PTRACE("KST %8.5f %8.5f %8.5f %8.5f %8.5f %8.5f", currPos.x, currVel.x, 0.0, newVel.x, 1.5, _dt);


    //TRACE("<");
}

