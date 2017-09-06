 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cShootPlanningInterface.cpp
 *
 *  Created on: Jan 12, 2016
 *      Author: Coen Tempelaars
 */

#include "int/adapters/cShootPlanningInterface.hpp"

#include "cShootPlanningNames.h"

#include "shootPlanning/s_shootplanning_get_active.h"
#include "shootPlanning/s_shootplanning_set_active.h"
#include "shootPlanning/s_shoot.h"
#include "shootPlanning/s_lobshot.h"

#include "int/utilities/trace.hpp"

cShootPlanningInterface::cShootPlanningInterface()
{

}

cShootPlanningInterface::~cShootPlanningInterface()
{

}

void cShootPlanningInterface::connect()
{
    ros::service::waitForService(ShootPlanningInterface::s_shootplanning_get_active);
    shootPlanningGetActiveService = n.serviceClient<shootPlanning::s_shootplanning_get_active>
                                                   (ShootPlanningInterface::s_shootplanning_get_active, true);

    ros::service::waitForService(ShootPlanningInterface::s_shootplanning_set_active);
    shootPlanningSetActiveService = n.serviceClient<shootPlanning::s_shootplanning_set_active>
                                                   (ShootPlanningInterface::s_shootplanning_set_active, true);

    ros::service::waitForService(ShootPlanningInterface::s_shoot);
    shootService = n.serviceClient<shootPlanning::s_shoot>
                                  (ShootPlanningInterface::s_shoot, true);

    ros::service::waitForService(ShootPlanningInterface::s_lobshot);
    lobShotService = n.serviceClient<shootPlanning::s_lobshot>
                                  (ShootPlanningInterface::s_lobshot, true);
}

void cShootPlanningInterface::disable()
{
    shootPlanning::s_shootplanning_set_active service_params;
    service_params.request.active = false;

    bool call_succeeded = shootPlanningSetActiveService.call(service_params);

    if (call_succeeded)
    {
        TRACE("shootplanning is now passive");
    }
    else
    {
        TRACE_ERROR("ROS call shootplanning set active failed.");
        connect();
    }
}

void cShootPlanningInterface::enable()
{
    shootPlanning::s_shootplanning_set_active service_params;
    service_params.request.active = true;

    bool call_succeeded = shootPlanningSetActiveService.call(service_params);

    if (call_succeeded)
    {
        TRACE("shootplanning is now active");
    }
    else
    {
        TRACE_ERROR("ROS call shootplanning set active failed.");
        connect();
    }
}

bool cShootPlanningInterface::isEnabled()
{
    bool is_enabled = false;
    shootPlanning::s_shootplanning_get_active service_params;

    bool call_succeeded = shootPlanningGetActiveService.call(service_params);

    if (call_succeeded)
    {
        is_enabled = (service_params.response.active != 0);
    }
    else
    {
        TRACE_ERROR("ROS call shootplanning get active failed (ignored, assuming shootplanning not active).");
        connect();
    }

    TRACE("shootplanning is ") << ((is_enabled)?("active"):("passive"));
    return is_enabled;
}

void cShootPlanningInterface::shoot(float strength)
{
    shootPlanning::s_shoot service_params;
    service_params.request.shootTarget.strength = strength;

    TRACE("shoot with strength: ") << std::to_string(strength);
    if(!shootService.call(service_params))
    {
    	connect();
    }
}

void cShootPlanningInterface::lobShot (const Point2D& target)
{
	//For now, no parameters yet for lobshot.
	//Shootplanning will calculate if its possible, if not a normal shot will be performed

    shootPlanning::s_lobshot service_params;
    //Shootplanning will determine the strength from the x and y coordinates
    service_params.request.shootTarget.strength = 0.0;
    service_params.request.shootTarget.target_x = target.x;
    service_params.request.shootTarget.target_y = target.y;
    service_params.request.shootTarget.target_z = 0.0;

    TRACE("lob shot service called");
    if(!lobShotService.call(service_params))
    {
    	connect();
    }
}

