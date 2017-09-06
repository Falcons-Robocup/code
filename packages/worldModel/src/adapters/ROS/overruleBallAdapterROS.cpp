 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * overruleBallAdapterROS.cpp
 *
 *  Created on: Nov 15, 2016
 *      Author: Tim Kouters
 */

#include "int/adapters/ROS/overruleBallAdapterROS.hpp"

#include "ext/WorldModelNames.h"

#include "cDiagnosticsEvents.hpp"
#include "FalconsCommon.h"
#include "timeConvert.hpp"

overruleBallAdapterROS::overruleBallAdapterROS()
{

}

overruleBallAdapterROS::~overruleBallAdapterROS()
/*
 * Chuck Norris can kill your imaginary friends
 */
{

}

void overruleBallAdapterROS::InitializeROS()
{
	try
	{
		_hROS.reset(new ros::NodeHandle());

		_srvOverruleBall = _hROS->advertiseService(
                WorldModelInterface::s_set_overrule_ball_location,
                &overruleBallAdapterROS::overrule_ball_location_cb, this);
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

bool overruleBallAdapterROS::overrule_ball_location_cb(
        		worldModel::overrule_ball_location::Request &req,
        		worldModel::overrule_ball_location::Response &resp)
{
	try
	{
		ballClass_t ball;

		ball.setCoordinates(
				req.positionX,
				req.positionY,
				req.positionZ);
		ball.setVelocities(
				req.velocityX,
				req.velocityY,
				req.velocityZ);
		ball.setId(getRobotNumber());
		ball.setTimestamp(getTimeNow() + req.overruleTimeInSeconds);
		ball.setConfidence(0.99);
		ball.setIsValid(true);

		notify(ball);

		return true;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}
