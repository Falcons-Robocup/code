 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPathPlanningInterface.cpp
 * ROS interface towards the pathplanning package
 *
 *  Created on: Dec 29, 2015
 *      Author: Coen Tempelaars
 */

#include "int/adapters/cPathPlanningInterface.hpp"

#include "cPathPlanningNames.hpp"

#include "pathPlanning/s_pathplanning_get_active.h"
#include "pathPlanning/s_pathplanning_set_active.h"
#include "rosMsgs/t_target.h"
#include "polygon2D.hpp"

#include "int/utilities/trace.hpp"


cPathPlanningInterface::cPathPlanningInterface()
{
}

cPathPlanningInterface::~cPathPlanningInterface()
{
}

void cPathPlanningInterface::connect()
{
    n.reset(new ros::NodeHandle());

    ros::service::waitForService(PathPlanningInterface::s_pathplanning_get_active);
    pathPlanningGetActive = n->serviceClient<pathPlanning::s_pathplanning_get_active>
                                           (PathPlanningInterface::s_pathplanning_get_active, true);

    ros::service::waitForService(PathPlanningInterface::s_pathplanning_get_active);
    pathPlanningSetActive = n->serviceClient<pathPlanning::s_pathplanning_set_active>
                                           (PathPlanningInterface::s_pathplanning_set_active, true);

    targetPublisher = n->advertise<rosMsgs::t_target>("g_target", 1);
}

void cPathPlanningInterface::disable()
{
    pathPlanning::s_pathplanning_set_active service_params;
    service_params.request.active = false;

    bool call_succeeded = pathPlanningSetActive.call(service_params);

    if (call_succeeded)
    {
        TRACE("pathplanning is now passive");
    }
    else
    {
        TRACE_ERROR("ROS call pathplanning set active failed.");
        connect();
    }
}

void cPathPlanningInterface::enable()
{
    pathPlanning::s_pathplanning_set_active service_params;
    service_params.request.active = true;

    bool call_succeeded = pathPlanningSetActive.call(service_params);

    if (call_succeeded)
    {
        TRACE("pathplanning is now active");
    }
    else
    {
        TRACE_ERROR("ROS call pathplanning set active failed.");
        connect();
    }
}

bool cPathPlanningInterface::isEnabled()
{
    bool is_enabled = false;
    pathPlanning::s_pathplanning_get_active service_params;

    bool call_succeeded = pathPlanningGetActive.call(service_params);

    if (call_succeeded)
    {
        is_enabled = (service_params.response.active != 0);
    }
    else
    {
        TRACE_ERROR("ROS call pathplanning get active failed (ignored, assuming pathplanning not active).");
        connect();
    }

    TRACE("pathplanning is ") << ((is_enabled)?("active"):("passive"));
    return is_enabled;
}

void cPathPlanningInterface::moveTo(geometry::Pose2D& pose2d, const std::vector<polygon2D>& forbiddenAreas, const std::string& motionProfile)
{
    uint8_t msg_motionProfile = 0;
    if (motionProfile.compare("normal") == 0)
    {
        msg_motionProfile = rosMsgs::t_target::MOTIONPROFILE_NORMAL;
    }
    else if (motionProfile.compare("setpiece") == 0)
    {
        msg_motionProfile = rosMsgs::t_target::MOTIONPROFILE_SETPIECE;
    }

    rosMsgs::t_target target_pos;
    target_pos.x = pose2d.getX();
    target_pos.y = pose2d.getY();
    target_pos.phi = pose2d.getPhi();
    target_pos.motion_profile = msg_motionProfile;
    target_pos.move_type = rosMsgs::t_target::MOVETYPE_NORMAL;
    TRACE("publishing target (") << std::to_string(target_pos.x)
    << ", " << std::to_string(target_pos.y)
    << ", " << std::to_string(target_pos.phi) << ")";

    int i = 0;
    for (auto forbiddenArea : forbiddenAreas)
    {
        rosMsgs::t_polygon2D ros_forbidden_area;
        ros_forbidden_area.id = i;

        std::vector<Point2D> points = forbiddenArea.getPoints();
        for(auto it = points.begin(); it != points.end(); it++)
        {
        	rosMsgs::t_point2D point;
        	point.x = it->x;
        	point.y = it->y;
        	ros_forbidden_area.points.push_back(point);
        }

        target_pos.forbiddenAreas.push_back(ros_forbidden_area);
        i++;
    }

    targetPublisher.publish(target_pos);
}

