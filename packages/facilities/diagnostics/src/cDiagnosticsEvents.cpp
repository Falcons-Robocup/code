 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cDiagnosticsEvents.cpp
 *
 */


#include "ext/cDiagnosticsEvents.hpp"
#include <ros/ros.h>
#include "rosMsgs/t_event.h"
#include "FalconsCommon.h"
#include "tracer.hpp"




// local declarations
void trace_and_publish(falcons::eventType const &event, int eventLevel);


void diagnostics::diag_info(falcons::eventType const &event)
{
    trace_and_publish(event, 0);
}

void diagnostics::diag_warning(falcons::eventType const &event)
{
    trace_and_publish(event, 1);
}

void diagnostics::diag_error(falcons::eventType const &event)
{
    trace_and_publish(event, 2);
}

void trace_and_publish(falcons::eventType const &event, int eventLevel)
{
    TRACE("got an event with level %d, see next line for its trace:", eventLevel);
    // first trace it (note: macro was expanded on original location)
    trace_write_event(event);
    // construct ROS message
    rosMsgs::t_event eventMsg;
    eventMsg.eventId = 0; // to be filled by robotControl, right before sending to coach
    eventMsg.robotId = getRobotNumber();
    eventMsg.fileName = event.fileName;
    eventMsg.funcName = event.funcName;
    eventMsg.lineNumber = event.lineNumber;
    eventMsg.timeStamp = event.timeStamp;
    eventMsg.eventType = eventLevel;
    eventMsg.eventString = event.message;
    // construct publisher, first time only
    bool _initFlag = false;
    static ros::NodeHandle *_nh = NULL;
    if (_nh == NULL)
    {
        if (ros::ok())
        {
            TRACE("making nh pointer");
            _nh = new ros::NodeHandle();
            TRACE("done making nh pointer");
        }
        else
        {
            // it seems client did not yet call ros::init ... too bad, probably we are very early in node construction
            // TODO: make a queue of these messages and a timer thread to flush them .... 
            TRACE("ros::init not yet called!!");
            return;
        }
        _initFlag = true;
    }
    TRACE("constructing publisher ok=%d", ros::ok());
    static ros::Publisher _publisher = _nh->advertise<rosMsgs::t_event>("g_event", 10);

    // give ROS some time to intialize... otherwise first message has a very high chance to get lost
    // http://answers.ros.org/question/11167/how-do-i-publish-exactly-one-message/
    if (_initFlag)
    {
        TRACE("sleeping due to initialization");
        sleep(2); 
    }

    // publish
    TRACE("publishing");
    _publisher.publish(eventMsg);
    TRACE("event has been published");
}

