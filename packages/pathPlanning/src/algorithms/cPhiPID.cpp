 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
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

#include "int/algorithms/cPhiPID.hpp"

void cPhiPID::execute()
{
    TRACE(">");
    // Get current position from cPathPlanningData
    Position2D currPos;
    _main->_ppData->getPosition(currPos);

    // Get PID parameters (reconfig) from cPathPlanningData
    pp_pid_params_struct_t pidParams;
    _main->_ppData->getPIDParams(pidParams);

    //plotdata
    pp_plot_data_struct_t plotdata;
    _main->_ppData->getPlotData(plotdata);

    double phierror = project_angle_mpi_pi(_ppData.pos.phi - currPos.phi);

    /* integral = integral + (error * dt) */
    _integral = _integral + phierror * _dt;

    if (_integral > pidParams.maxI_Phi)
    {
        _integral = pidParams.maxI_Phi;
    }
    else if (_integral < -(pidParams.maxI_Phi))
    {
        _integral = -(pidParams.maxI_Phi);
    }

    /* derivative = (error - previous_error) / dt */
    double phiderivative = (phierror - _prev_vel.phi) / _dt;

    Velocity2D newVel;
    newVel.phi = pidParams.PHI_P * phierror
                   + pidParams.PHI_I * _integral
                   + pidParams.PHI_D * phiderivative;

    _prev_vel.phi = phierror;

    _ppData.vel.phi = newVel.phi;

    std::stringstream ss;
    ss << "PHI PIDPLOT " << "Error: " << phierror << "PhiError: " << (pidParams.PHI_P * phierror) << "PhiIntegral: " << (pidParams.PHI_I * _integral) << "PhiDerivative: " << (pidParams.PHI_D * phiderivative);
    TRACE(ss.str().c_str());

    plotdata.phi_pid_out = newVel.phi;
    plotdata.phi_pid_p = pidParams.PHI_P * phierror;
    plotdata.phi_pid_i = pidParams.PHI_I * _integral;
    plotdata.phi_pid_d = pidParams.PHI_D * phiderivative;
    _main->_ppData->setPlotData(plotdata);

    TRACE("<");
}


