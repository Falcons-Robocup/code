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

#include "tracing.hpp"

#include "int/adapters/cRTDBOutputAdapter.hpp"

cRTDBOutputAdapter::cRTDBOutputAdapter(PeripheralsInterfaceData& piData)
    : _piData(piData)
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId);
    TRACE("<");
}

cRTDBOutputAdapter::~cRTDBOutputAdapter()
{
}

void cRTDBOutputAdapter::setMotorFeedback()
{
    TRACE_FUNCTION("");

    T_MOTOR_FEEDBACK motorFeedback;

    piDisplacement disp = _piData.getDisplacementOutput();
    motorFeedback.m1.displacement = disp.m1_pos;
    motorFeedback.m2.displacement = disp.m2_pos;
    motorFeedback.m3.displacement = disp.m3_pos;

    piVelAcc vel = _piData.getVelocityOutput();
    motorFeedback.m1.velocity = vel.m1_vel;
    motorFeedback.m2.velocity = vel.m2_vel;
    motorFeedback.m3.velocity = vel.m3_vel;

    _rtdb->put(MOTOR_FEEDBACK, &motorFeedback);
}

void cRTDBOutputAdapter::setBallHandlersFeedback()
{
    TRACE_FUNCTION("");

    T_BALLHANDLERS_FEEDBACK ballHandlersFeedback;

    BallhandlerFeedback feedback = _piData.getBallhandlerFeedback();
    ballHandlersFeedback.angleLeft = feedback.angleLeft;
    ballHandlersFeedback.angleRight = feedback.angleRight;
    ballHandlersFeedback.velocityLeft = feedback.velocityLeft;
    ballHandlersFeedback.velocityRight = feedback.velocityRight;

    _rtdb->put(BALLHANDLERS_FEEDBACK, &ballHandlersFeedback);
}

void cRTDBOutputAdapter::setInPlayFeedback()
{
    TRACE_FUNCTION("");

    T_INPLAY_FEEDBACK inPlayFeedback;
    //_piData.

    _rtdb->put(INPLAY_FEEDBACK, &inPlayFeedback);
}
