 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRTDBOutputAdapter.cpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Erik Kouters
 */

#include "tracing.hpp"

#include "int/adapters/cRTDBOutputAdapter.hpp"

cRTDBOutputAdapter::cRTDBOutputAdapter()
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId);
    TRACE("<");
}

cRTDBOutputAdapter::~cRTDBOutputAdapter()
{
}

void cRTDBOutputAdapter::setRobotDisplacementFeedback(const vc_robot_data& robotData)
{
    std::stringstream str;
    str << "x=" << robotData.displacement.x << "; y=" << robotData.displacement.y << "; Rz=" << project_angle_mpi_pi(robotData.displacement.phi);
    TRACE_FUNCTION(str.str().c_str());

    // Put the latest displacement to RTDB

    T_ROBOT_DISPLACEMENT_FEEDBACK robotDisplacementFeedback;

    robotDisplacementFeedback.x = robotData.displacement.x;
    robotDisplacementFeedback.y = robotData.displacement.y;
    robotDisplacementFeedback.Rz = project_angle_mpi_pi(robotData.displacement.phi);
    _rtdb->put(ROBOT_DISPLACEMENT_FEEDBACK, &robotDisplacementFeedback);
}

void cRTDBOutputAdapter::setRobotVelocityFeedback(const vc_robot_data& robotData)
{
    std::stringstream str;
    str << "x=" << robotData.velocity.x << "; y=" << robotData.velocity.y << "; Rz=" << project_angle_mpi_pi(robotData.velocity.phi);
    TRACE_FUNCTION(str.str().c_str());

    // Get the current list of velocities from RTDB
    // Add the latest velocity to the list
    // Put the list back to RTDB

    T_ROBOT_VELOCITY_FEEDBACK robotVelocityFeedback;

    robotVelocityFeedback.x = robotData.velocity.x;
    robotVelocityFeedback.y = robotData.velocity.y;
    robotVelocityFeedback.Rz = robotData.velocity.phi;

    _rtdb->put(ROBOT_VELOCITY_FEEDBACK, &robotVelocityFeedback);
}

void cRTDBOutputAdapter::setMotorVelocitySetpoint(const vc_motors_data& motorsData)
{
    std::stringstream str;
    str << "m1=" << motorsData.m1.velocity << "; m2=" << motorsData.m2.velocity << "; m3=" << motorsData.m3.velocity;
    TRACE_FUNCTION(str.str().c_str());

    T_MOTOR_VELOCITY_SETPOINT motorVelocitySetpoint;
    motorVelocitySetpoint.m1 = motorsData.m1.velocity;
    motorVelocitySetpoint.m2 = motorsData.m2.velocity;
    motorVelocitySetpoint.m3 = motorsData.m3.velocity;

    _rtdb->put(MOTOR_VELOCITY_SETPOINT, &motorVelocitySetpoint);
}
