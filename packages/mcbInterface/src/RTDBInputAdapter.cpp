 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #include <stdexcept>

#include "falconsCommon.hpp"
#include "tracing.hpp"

#include "RTDBInputAdapter.hpp"

using std::runtime_error;

RTDBInputAdapter::RTDBInputAdapter()
{
    TRACE_FUNCTION("");

    int robot_id = getRobotNumber();
    rtdb = RtDB2Store::getInstance().getRtDB2(robot_id);
}

RTDBInputAdapter::~RTDBInputAdapter()
{
}

::motors::RobotVector RTDBInputAdapter::getRobotVelocitySetpoint()
{
    TRACE_FUNCTION("");

    T_ROBOT_VELOCITY_SETPOINT robot_velocity_setpoint;
    int r = rtdb->get(ROBOT_VELOCITY_SETPOINT, &robot_velocity_setpoint);
    if (r != RTDB2_SUCCESS) {
        throw(runtime_error("Failed to retrieve robot velocity setpoint."));
    }

    ::motors::RobotVector velocity_setpoint;
    velocity_setpoint.set_x(robot_velocity_setpoint.x);
    velocity_setpoint.set_y(robot_velocity_setpoint.y);
    velocity_setpoint.set_phi(robot_velocity_setpoint.Rz);

    return velocity_setpoint;
}

::motors::BallhandlerAngles RTDBInputAdapter::getBallhandlerAngleSetpoints()
{
    TRACE_FUNCTION("");

    T_BALLHANDLERS_MOTOR_SETPOINT ballhandler_motor_setpoint;
    int r = rtdb->get(BALLHANDLERS_MOTOR_SETPOINT, &ballhandler_motor_setpoint);
    if (r != RTDB2_SUCCESS) {
        throw(runtime_error("Failed to retrieve ballhandler motor setpoint."));
    }

    ::motors::BallhandlerAngles ballhandler_angles;
    ballhandler_angles.set_left(ballhandler_motor_setpoint.bhMotorData.angleLeft);
    ballhandler_angles.set_right(ballhandler_motor_setpoint.bhMotorData.angleRight);

    return ballhandler_angles;
}

bool RTDBInputAdapter::getBallhandlersOn()
{
    TRACE_FUNCTION("");

    T_BALLHANDLERS_MOTOR_SETPOINT ballhandler_motor_setpoint;
    int r = rtdb->get(BALLHANDLERS_MOTOR_SETPOINT, &ballhandler_motor_setpoint);
    if (r != RTDB2_SUCCESS) {
        throw(runtime_error("Failed to retrieve ballhandler motor setpoint."));
    }

    return ballhandler_motor_setpoint.enabled;
}

void RTDBInputAdapter::waitForKickerSetpoint()
{
    rtdb->waitForPut(KICKER_SETPOINT);
}

KickerSetpoint RTDBInputAdapter::getKickerSetpoint()
{
    TRACE_FUNCTION("");

    T_KICKER_SETPOINT kicker_setpoint;
    int r = rtdb->get(KICKER_SETPOINT, &kicker_setpoint);
    if (r != RTDB2_SUCCESS) {
        throw(runtime_error("Failed to retrieve kicker setpoint."));
    }

    KickerSetpoint setpoint;
    switch (kicker_setpoint.kickerSetpointType)
    {
        case kickerSetpointTypeEnum::HOME:
        setpoint.type = KickerSetpoint::Type::Home;
            break;
        case kickerSetpointTypeEnum::SET_HEIGHT:
        setpoint.type = KickerSetpoint::Type::SetHeight;
        setpoint.value = kicker_setpoint.kickerHeight / 255.0;
            break;
        case kickerSetpointTypeEnum::SHOOT:
        setpoint.type = KickerSetpoint::Type::Shoot;
        setpoint.value = kicker_setpoint.kickerPower / 255.0;
            break;
    }

    return setpoint;
}
