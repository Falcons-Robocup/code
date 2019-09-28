 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
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

#include "int/adapters/cRTDBInputAdapter.hpp"

cRTDBInputAdapter::cRTDBInputAdapter(PeripheralsInterfaceData& piData) :
    _piData(piData)
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId);
    TRACE("<");
}

cRTDBInputAdapter::~cRTDBInputAdapter()
{
}

void cRTDBInputAdapter::getMotorVelocitySetpoint()
{
    TRACE_FUNCTION("");
    T_MOTOR_VELOCITY_SETPOINT motorVelSetpoint;
    int ageMs = 0;

    int r = _rtdb->get(MOTOR_VELOCITY_SETPOINT, &motorVelSetpoint, ageMs, _myRobotId);
    int watchDogTimeoutMs = 150;

    // we need watchdog behavior in case any process breaks the execution architecture stream
    // motor velocity setpoint would freeze in that case, so we should to not relay it to motors, better to hit the brakes then!
    piVelAcc vel;
    vel.m1_vel = 0.0;
    vel.m2_vel = 0.0;
    vel.m3_vel = 0.0;
    if ((r == RTDB2_SUCCESS) && (ageMs < watchDogTimeoutMs))
    {
        vel.m1_vel = motorVelSetpoint.m1;
        vel.m2_vel = motorVelSetpoint.m2;
        vel.m3_vel = motorVelSetpoint.m3;

    }
    _piData.setVelocityInput(vel);
}

void cRTDBInputAdapter::getBallHandlersMotorSetpoint()
{
    TRACE_FUNCTION("");
    T_BALLHANDLERS_MOTOR_SETPOINT ballHandlersMotorSetpoint;
    int ageMs = 0;

    int r = _rtdb->get(BALLHANDLERS_MOTOR_SETPOINT, &ballHandlersMotorSetpoint, ageMs, _myRobotId);

    if (r == RTDB2_SUCCESS)
    {
        // BH Enabled / Disabled
        BallhandlerSettings settings = _piData.getBallhandlerSettings();
        if (ballHandlersMotorSetpoint.enabled)
        {
            settings.controlMode = BallhandlerBoardControlMode::BALLHANDLER_CONTROL_MODE_ON;
        }
        else
        {
            settings.controlMode = BallhandlerBoardControlMode::BALLHANDLER_CONTROL_MODE_OFF;
        }
        _piData.setBallhandlerSettings(settings);

        // Angle and Velocity Setpoints
        BallhandlerSetpoints setpoints;
        setpoints.angleLeft = ballHandlersMotorSetpoint.bhMotorData.angleLeft;
        setpoints.angleRight = ballHandlersMotorSetpoint.bhMotorData.angleRight;
        setpoints.velocityLeft = ballHandlersMotorSetpoint.bhMotorData.velocityLeft;
        setpoints.velocityRight = ballHandlersMotorSetpoint.bhMotorData.velocityRight;
        _piData.setBallhandlerSetpoints(setpoints);
    }

}
