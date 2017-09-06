 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cKickerAdapter.cpp
 *
 *  Created on: Jun 21, 2015
 *      Author: Tim Kouters
 */

#include <stdexcept>
#include <cDiagnosticsEvents.hpp>

#include "int/cKickerAdapter.hpp"
#include "int/cShootPlanner.hpp"
#include "ext/cShootPlanningNames.h"
#include "peripheralsInterface/s_peripheralsInterface_setKickSpeed.h"
#include "peripheralsInterface/s_peripheralsInterface_setKickPosition.h"
#include <WorldModelNames.h>
#include "FalconsCommon.h"

using std::string;
using std::exception;
using std::runtime_error;

cKickerAdapter::cKickerAdapter()
{
	string kick_position_service_name = ShootPlanningInterface::s_kick_position;
	string kick_speed_service_name = ShootPlanningInterface::s_kick_speed;

	ros::service::waitForService(kick_position_service_name);
	ros::service::waitForService(kick_speed_service_name);

	_cKickerHeight = _hROS.serviceClient<peripheralsInterface::s_peripheralsInterface_setKickPosition>(
			kick_position_service_name, USE_SERVICE_PERSISTENCY);
	_cKickerSpeed = _hROS.serviceClient<peripheralsInterface::s_peripheralsInterface_setKickSpeed>(
			kick_speed_service_name, USE_SERVICE_PERSISTENCY);
}

cKickerAdapter::~cKickerAdapter()
{

}

void cKickerAdapter::kickBall(double speed, bool &kickSuccessful)
{
    peripheralsInterface::s_peripheralsInterface_setKickSpeed srvKickSpeed;
    //TODO worldModel::get_ball_possession srv_ball_possession;

    try
    {
        TRACE_INFO("kicking with speed %.1f", speed);

        kickSuccessful = false;
        srvKickSpeed.request.kick_speed = speed;

        if(!_cKickerSpeed.call(srvKickSpeed))
        {
            TRACE_ERROR("Failed to request kick speed");
            kickSuccessful = false;
        }
        else
        {
            kickSuccessful = true;
        }

    } catch(exception &e)
    {
        throw e;
    }
}

void cKickerAdapter::setHeightKicker(double height, bool &successful)
{
    peripheralsInterface::s_peripheralsInterface_setKickPosition srvKickHeight;

    try
    {

        successful = false;

        TRACE("Set kicker height to %f", height);
        srvKickHeight.request.kick_position = height;


        if (!_cKickerHeight.call(srvKickHeight))
        {
            successful = false;
            TRACE_ERROR("Service failure for requesting Kick Position");
        }
        else
        {
            successful = true;
        }

    } catch (exception &e)
    {
        throw e;
    }
}
