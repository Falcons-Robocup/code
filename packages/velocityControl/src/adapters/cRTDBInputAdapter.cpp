 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRTDBInputAdapter.cpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Erik Kouters
 */

#include "tracing.hpp"
#include "cDiagnostics.hpp"

#include "int/adapters/cRTDBInputAdapter.hpp"

cRTDBInputAdapter::cRTDBInputAdapter(cVelocityControlData *data, iterateFunctionType feedbackfunc, iterateFunctionType setpointfunc)
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId);
    _vcData = data;
    _iterateFeedbackFunc = feedbackfunc;
    _iterateSetpointFunc = setpointfunc;
    TRACE("<");
}

cRTDBInputAdapter::~cRTDBInputAdapter()
{
}

void cRTDBInputAdapter::waitForMotorFeedback()
{
    while (true)
    {
        try
        {
            _rtdb->waitForPut(MOTOR_FEEDBACK);

            getMotorFeedback();

            _iterateFeedbackFunc();
        }
        catch(std::exception &e)
        {
            TRACE_ERROR("cRTDBInputAdapter::waitForMotorFeedback() failed: %s", e.what());
            std::cout << "cRTDBInputAdapter::waitForMotorFeedback() failed: " << e.what() << std::endl;
        }
    }
}

void cRTDBInputAdapter::waitForRobotVelocitySetpoint()
{
    while (true)
    {
        try
        {
            _rtdb->waitForPut(ROBOT_VELOCITY_SETPOINT);

            getRobotVelocitySetpoint();

            _iterateSetpointFunc();
        }
        catch(std::exception &e)
        {
            TRACE_ERROR("cRTDBInputAdapter::waitForRobotVelocitySetpoint() failed: %s", e.what());
            std::cout << "cRTDBInputAdapter::waitForRobotVelocitySetpoint() failed: " << e.what() << std::endl;
        }
    }
}

void cRTDBInputAdapter::getRobotVelocitySetpoint()
{
    TRACE_FUNCTION("");
    T_ROBOT_VELOCITY_SETPOINT robotVelSetpoint;

    int r = _rtdb->get(ROBOT_VELOCITY_SETPOINT, &robotVelSetpoint);

    if (r == RTDB2_SUCCESS)
    {
        tprintf("get ROBOT_VELOCITY_SETPOINT [%6.2f, %6.2f, %6.2f]", robotVelSetpoint.x, robotVelSetpoint.y, robotVelSetpoint.Rz);
        vc_robot_data robotData;
        robotData.velocity.x = robotVelSetpoint.x;
        robotData.velocity.y = robotVelSetpoint.y;
        robotData.velocity.phi = robotVelSetpoint.Rz;
        _vcData->setTargetRobotData(robotData);
    }
}

void cRTDBInputAdapter::getMotorFeedback()
{
    TRACE_FUNCTION("");
    T_MOTOR_FEEDBACK motorFeedback;

    int r = _rtdb->get(MOTOR_FEEDBACK, &motorFeedback);

    if (r == RTDB2_SUCCESS)
    {
        tprintf("get MOTOR_FEEDBACK disp[%6.2f, %6.2f, %6.2f] vel[%6.2f, %6.2f, %6.2f]", motorFeedback.m1.displacement, motorFeedback.m2.displacement, motorFeedback.m3.displacement, motorFeedback.m1.velocity, motorFeedback.m2.velocity, motorFeedback.m3.velocity);
        vc_motors_data motorData;
        motorData.m1.displacement = motorFeedback.m1.displacement;
        motorData.m2.displacement = motorFeedback.m2.displacement;
        motorData.m3.displacement = motorFeedback.m3.displacement;
        motorData.m1.velocity = motorFeedback.m1.velocity;
        motorData.m2.velocity = motorFeedback.m2.velocity;
        motorData.m3.velocity = motorFeedback.m3.velocity;
        _vcData->setFeedbackMotorsData(motorData);
    }

}
