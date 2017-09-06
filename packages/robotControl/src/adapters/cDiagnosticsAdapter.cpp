 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cDiagnosticsAdapter.cpp
 *
 *  Created on: Jan 11, 2016
 *      Author: Jan Feitsma
 */

// system includes
#include <stdexcept>

// falcons external includes
#include "FalconsCommon.h"
#include "timeConvert.hpp"

// package internal includes
#include "int/adapters/cDiagnosticsAdapter.hpp"

#define DIAG_FREQUENCY 5.0
#define REPEAT_TIMEOUT 5.0

boost::mutex _gmtx; // around read/write on _recentEvents

using namespace std;

cDiagnosticsAdapterEventRelay::cDiagnosticsAdapterEventRelay()
 : diagnostics::cDiagnosticsSender<rosMsgs::t_diag_events>(diagnostics::DIAG_EVENTS, 0, true)
{
    TRACE("constructing cDiagnosticsAdapterEventRelay");
    _eventId = 0;
    _msgId = 0;
    // construct ros topic listener to g_events
    _subEvent = ros::NodeHandle().subscribe("g_event", 10, &cDiagnosticsAdapterEventRelay::eventCallback, this);
    // construct the timed thread for triggering the event communication 
    _thread = boost::thread(&cDiagnosticsAdapterEventRelay::timedThread, this);
    TRACE("cDiagnosticsAdapterEventRelay constructed OK");
}    

void cDiagnosticsAdapterEventRelay::eventCallback(const rosMsgs::t_event::ConstPtr& eventPtr)
{
    // push the event in the deque
    _gmtx.lock();
    _recentEvents.push_front(*eventPtr);
    // fill in the unique eventId
    _recentEvents[0].eventId = _eventId++;
    // trace the delay per event -- only for first event per process we expect a
    // delay of 2.0 seconds, because of the time needed to get the topic constructed & registered at ROS
    // subsequent events should arrive immediately here
    TRACE("stored eventId=%d, delay=%.1fs", _recentEvents[0].eventId, getTimeNow() - _recentEvents[0].timeStamp);
    _gmtx.unlock();
    // trigger send
    updateAndSend();
}

void cDiagnosticsAdapterEventRelay::updateAndSend()
{
    _gmtx.lock();
    // clear out old events, also drop spammed events
    size_t maxBufferSize = 10;
    size_t currentBufferSize = _recentEvents.size();
    double t = getTimeNow() - REPEAT_TIMEOUT;
    while (_recentEvents.size() && _recentEvents.back().timeStamp < t)
    {
        _recentEvents.pop_back();
    }
    while (_recentEvents.size() > maxBufferSize)
    {
        _recentEvents.pop_back();
    }
    // construct message
    rosMsgs::t_diag_events msg;
    for (auto it = _recentEvents.begin(); it != _recentEvents.end(); ++it)
    {
        msg.events.push_back(*it);
    }
    msg.id = _msgId++;
    msg.timeStamp = getTimeNow();
    // send message
    TRACE("send event dropped=%d msgId=%d", (int)(_recentEvents.size() - currentBufferSize), msg.id);
    set(msg);
    _gmtx.unlock();
}

void cDiagnosticsAdapterEventRelay::timedThread()
{
    float dt = 1.0 / DIAG_FREQUENCY;
    while (true)
    {
        updateAndSend();
        // sleep
        ros::Duration(dt).sleep();
    }
}


