 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * wmSyncAdapterTeamMemberROS.cpp
 *
 *  Created on: Nov 20, 2016
 *      Author: Tim Kouters
 */

#include "int/adapters/ROS/wmSyncAdapterTeamMemberROS.hpp"

#include "int/facilities/ROSConvert.hpp"

#include "ext/WorldModelNames.h"

#include "cDiagnosticsEvents.hpp"

wmSyncAdapterTeamMemberROS::wmSyncAdapterTeamMemberROS()
{

}

wmSyncAdapterTeamMemberROS::~wmSyncAdapterTeamMemberROS()
/*
 * Chuck Norris walked into a bar
 * The bar said "ouch"
 */
{

}

void wmSyncAdapterTeamMemberROS::InitializeROS()
{
	try
	{
		_hROS.reset(new ros::NodeHandle());

		_srvTeamMemberLocation = _hROS->advertiseService(
                WorldModelInterface::s_set_member_location,
                &wmSyncAdapterTeamMemberROS::set_member_location_cb, this);

	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

bool wmSyncAdapterTeamMemberROS::set_member_location_cb(
        		worldModel::set_member_location::Request &req,
        		worldModel::set_member_location::Response &resp)
{
	try
	{
		robotClass_t robot;

		robot.setCoordinateType(coordinateType::FIELD_COORDS);
		robot.setRobotID(req.robotID);
		robot.setTimestamp(req.robotPos.timestamp);
		robot.setCoordinates(
				req.robotPos.positionX,
				req.robotPos.positionY,
				req.robotPos.positionTheta);
		robot.setVelocities(
				req.robotPos.velocityX,
				req.robotPos.velocityY,
				req.robotPos.velocityTheta
				);

		notify(robot);

		return true;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}
