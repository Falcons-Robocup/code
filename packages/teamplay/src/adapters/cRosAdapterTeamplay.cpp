 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRosAdapterTeamplay.cpp
 *
 *  Created on: Feb 4, 2016
 *      Author: Coen Tempelaars
 */

#include "int/cRosAdapterTeamplay.hpp"
#include "int/cTeamplayControlInterface.hpp"
#include "int/cTeamplayServices.hpp"

#include "int/utilities/trace.hpp"

#include "teamplay/s_park.h"
#include "teamplay/s_set_orientation.h"
#include "teamplay/s_stop.h"
#include "teamplay/s_teamplay_get_active.h"
#include "teamplay/s_teamplay_set_active.h"


cTeamplayControlInterface* _tp;


bool park_service_callback (teamplay::s_park::Request& req, teamplay::s_park::Response& resp)
{
    TRACE("teamplay: park");
    _tp->park();
    return true;
}

bool set_orientation_service_callback (teamplay::s_set_orientation::Request& req,
                                       teamplay::s_set_orientation::Response& resp)
{
    switch (req.orientation)
    {
    case req.ORIENTATION_LEFT_TO_RIGHT:
        TRACE("teamplay: set orientation left to right");
        _tp->setOrientation(orientationEnum::LEFT_TO_RIGHT);
        return true;

    case req.ORIENTATION_RIGHT_TO_LEFT:
        TRACE("teamplay: set orientation right to left");
        _tp->setOrientation(orientationEnum::RIGHT_TO_LEFT);
        return true;

    default:
        TRACE_ERROR("teamplay: set orientation: illegal parameter: ") << std::to_string(req.orientation));
        return false;
    }
}

bool stop_service_callback (teamplay::s_stop::Request& req, teamplay::s_stop::Response& resp)
{
    TRACE("teamplay: stop");
    _tp->stop();
    return true;
}

bool teamplay_get_active_callback (teamplay::s_teamplay_get_active::Request& req,
                                   teamplay::s_teamplay_get_active::Response& resp)
{
    bool active = _tp->isActive();
    TRACE("teamplay: get active: teamplay is ") << std::to_string(((active)?("active"):("passive"));
    resp.active = active;
    return true;
}

bool teamplay_set_active_callback (teamplay::s_teamplay_set_active::Request& req,
                                   teamplay::s_teamplay_set_active::Response& resp)
{
    if (req.active)
    {
        TRACE("teamplay: set active: teamplay is now active");
        _tp->enable();
    }
    else
    {
        TRACE("teamplay: set active: teamplay is now passive");
        _tp->disable();
    }
    return true;
}

cRosAdapterTeamplay::cRosAdapterTeamplay(cTeamplayControlInterface* tp)
{
    _tp = tp;
    registerAllServices();
}

void cRosAdapterTeamplay::registerAllServices()
{
    TRACE("teamplay: setting up services");
    _services.push_back(_nh.advertiseService(TeamplayServices::s_park, park_service_callback));
    _services.push_back(_nh.advertiseService(TeamplayServices::s_set_orientation, set_orientation_service_callback));
    _services.push_back(_nh.advertiseService(TeamplayServices::s_stop, stop_service_callback));
    _services.push_back(_nh.advertiseService(TeamplayServices::s_teamplay_get_active, teamplay_get_active_callback));
    _services.push_back(_nh.advertiseService(TeamplayServices::s_teamplay_set_active, teamplay_set_active_callback));
    TRACE("teamplay: done setting up services");
}


