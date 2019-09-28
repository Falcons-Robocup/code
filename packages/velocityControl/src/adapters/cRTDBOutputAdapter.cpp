 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
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

#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

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
    TRACE_FUNCTION("");

    // Get the current list of displacements from RTDB
    // Add the latest displacement to the list
    // Put the list back to RTDB

    T_ROBOT_DISPLACEMENT_FEEDBACK robotDisplacementFeedback;
    int ageMs = 0;

    // mutex with reading from WorldModel
    tprintf("setRobotDisplacementFeedback before lock");
    boost::interprocess::named_mutex mtx(boost::interprocess::open_or_create, "robot_displacement_named_mutex");
    {
        boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(mtx);

        _rtdb->get(ROBOT_DISPLACEMENT_FEEDBACK, &robotDisplacementFeedback, ageMs, _myRobotId);
        tprintf("setRobotDisplacementFeedback robotDisplacementFeedback.size=%d", (int)robotDisplacementFeedback.size());

        robotDisplacement newDisplacement;
        newDisplacement.x = robotData.displacement.x;
        newDisplacement.y = robotData.displacement.y;
        newDisplacement.Rz = project_angle_mpi_pi(robotData.displacement.phi);
        robotDisplacementFeedback.push_back(newDisplacement);

        tprintf("setRobotDisplacementFeedback put");
        _rtdb->put(ROBOT_DISPLACEMENT_FEEDBACK, &robotDisplacementFeedback);
    }
    tprintf("setRobotDisplacementFeedback end");
}

void cRTDBOutputAdapter::setRobotVelocityFeedback(const vc_robot_data& robotData)
{
    TRACE_FUNCTION("");

    // Get the current list of velocities from RTDB
    // Add the latest velocity to the list
    // Put the list back to RTDB

    T_ROBOT_VELOCITY_FEEDBACK robotVelocityFeedback;
    int ageMs = 0;

    robotVelocityFeedback.x = robotData.velocity.x;
    robotVelocityFeedback.y = robotData.velocity.y;
    robotVelocityFeedback.Rz = robotData.velocity.phi;

    _rtdb->put(ROBOT_VELOCITY_FEEDBACK, &robotVelocityFeedback);
}

void cRTDBOutputAdapter::setMotorVelocitySetpoint(const vc_motors_data& motorsData)
{
    TRACE_FUNCTION("");

    T_MOTOR_VELOCITY_SETPOINT motorVelocitySetpoint;
    motorVelocitySetpoint.m1 = motorsData.m1.velocity;
    motorVelocitySetpoint.m2 = motorsData.m2.velocity;
    motorVelocitySetpoint.m3 = motorsData.m3.velocity;

    _rtdb->put(MOTOR_VELOCITY_SETPOINT, &motorVelocitySetpoint);
}
