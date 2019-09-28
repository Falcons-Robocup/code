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

cRTDBInputAdapter::cRTDBInputAdapter(ballHandlingControl *bhControl)
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId);
    _bhControl = bhControl;
    TRACE("<");
}

cRTDBInputAdapter::~cRTDBInputAdapter()
{
}

void cRTDBInputAdapter::waitForBallHandlersSetpoint()
{
    while (true)
    {
        _rtdb->waitForPut(BALLHANDLERS_SETPOINT);
        getBallHandlersSetpoint();

        _bhControl->updateSetpoint();
    }
}

void cRTDBInputAdapter::waitForBallHandlersFeedback()
{
    while (true)
    {
        _rtdb->waitForPut(BALLHANDLERS_FEEDBACK);
        getBallHandlersFeedback();
        getRobotVelocityFeedback();

        _bhControl->updateSetpoint();
        _bhControl->updateFeedback();
    }
}

void cRTDBInputAdapter::getBallHandlersSetpoint()
{
    TRACE_FUNCTION("");
    T_BALLHANDLERS_SETPOINT ballHandlersSetpoint;

    int r = _rtdb->get(BALLHANDLERS_SETPOINT, &ballHandlersSetpoint);

    if (r == RTDB2_SUCCESS)
    {
        _bhControl->update_enabled(ballHandlersSetpoint);
    }
}

void cRTDBInputAdapter::getBallHandlersFeedback()
{
    TRACE_FUNCTION("");
    T_BALLHANDLERS_FEEDBACK ballHandlersFeedback;

    int r = _rtdb->get(BALLHANDLERS_FEEDBACK, &ballHandlersFeedback);

    if (r == RTDB2_SUCCESS)
    {
        ballHandlersStatusType bhStatus;
        bhStatus.angleLeft = ballHandlersFeedback.angleLeft;
        bhStatus.angleRight = ballHandlersFeedback.angleRight;
        bhStatus.velocityLeft = ballHandlersFeedback.velocityLeft;
        bhStatus.velocityRight = ballHandlersFeedback.velocityRight;

        _bhControl->update_status(bhStatus);
    }
}

void cRTDBInputAdapter::getRobotVelocityFeedback()
{
    TRACE_FUNCTION("");
    T_ROBOT_VELOCITY_FEEDBACK robotVelocityFeedback;

    int r = _rtdb->get(ROBOT_VELOCITY_FEEDBACK, &robotVelocityFeedback);

    if (r == RTDB2_SUCCESS)
    {
        Velocity2D robotVel = Velocity2D(robotVelocityFeedback.x, robotVelocityFeedback.y, robotVelocityFeedback.Rz);

        _bhControl->update_robot_velocity(robotVel);
    }

}

