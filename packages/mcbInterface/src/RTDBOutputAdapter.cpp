 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #include "falconsCommon.hpp"
#include "tracing.hpp"

#include "RTDBOutputAdapter.hpp"

RTDBOutputAdapter::RTDBOutputAdapter()
{
    TRACE_FUNCTION("");

    int robot_id = getRobotNumber();
    rtdb = RtDB2Store::getInstance().getRtDB2(robot_id);
}

RTDBOutputAdapter::~RTDBOutputAdapter()
{
}

void RTDBOutputAdapter::setRobotVelocity(const ::motors::RobotVector& velocity)
{
    TRACE_FUNCTION("");

    T_ROBOT_VELOCITY_FEEDBACK velocity_feedback;
    velocity_feedback.x = velocity.x();
    velocity_feedback.y = velocity.y();
    velocity_feedback.Rz = velocity.phi();
    rtdb->put(ROBOT_VELOCITY_FEEDBACK, &velocity_feedback);
}

void RTDBOutputAdapter::setRobotPosition(const ::motors::RobotVector& position)
{
    TRACE_FUNCTION("");

    T_ROBOT_DISPLACEMENT_FEEDBACK displacement_feedback;
    displacement_feedback.x = position.x();
    displacement_feedback.y = position.y();
    displacement_feedback.Rz = position.phi();
    rtdb->put(ROBOT_DISPLACEMENT_FEEDBACK, &displacement_feedback);
}

void RTDBOutputAdapter::setBallHandlerAngles(const ::motors::BallhandlerAngles& angles)
{
    TRACE_FUNCTION("");

    T_BALLHANDLERS_FEEDBACK ballhandlers_feedback;
    ballhandlers_feedback.angleLeft = angles.left();
    ballhandlers_feedback.angleRight = angles.right();
    rtdb->put(BALLHANDLERS_FEEDBACK, &ballhandlers_feedback);
}

void RTDBOutputAdapter::setInPlayStatus(bool in_play)
{
    TRACE_FUNCTION("");

    T_INPLAY_FEEDBACK inplay_feedback;
    if (in_play) {
        inplay_feedback = robotStatusEnum::INPLAY;
    }
    else {
        inplay_feedback = robotStatusEnum::OUTOFPLAY;
    }

    rtdb->put(INPLAY_FEEDBACK, &inplay_feedback);
}
