 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * administratorConfigROS.cpp
 *
 *  Created on: Nov 22, 2016
 *      Author: Tim Kouters
 */

#include "int/adapters/configurators/administratorConfigROS.hpp"

#include "int/configurators/administrationConfigurator.hpp"

#include <ros/ros.h>
#include <tracer.hpp>
#include <FalconsCommon.h> // for loadConfig

#include "cDiagnosticsEvents.hpp"

administratorConfigROS::administratorConfigROS()
{

}

administratorConfigROS::~administratorConfigROS()
/*
 * Chuck Norris invented Chuck Norris jokes,
 * but he never submitted any because Chuck Norris submits to no one
 */
{

}

void administratorConfigROS::loadConfigYaml()
{
	try
	{
	    loadConfig("worldModelAdministratorConfig");
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void administratorConfigROS::initializeROS()
{
	try
	{
		loadConfigYaml();

		_srvConfig.reset(new dynamic_reconfigure::Server<worldModel::administrationConfig>(ros::NodeHandle("~/administration")));

		/* Bind the reconfiguration function */
		_f = boost::bind(&administratorConfigROS::administratorConfigROS_cb, this, _1, _2);

		/* Start the server with the callback; this will automatically reconfigure the settings. */
		_srvConfig->setCallback(_f);

		forceReload();
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void administratorConfigROS::administratorConfigROS_cb(worldModel::administrationConfig &config, uint32_t level)
{
	TRACE("callback triggered");
	try
	{
		administrationConfigurator::getInstance().setBallTimeToLive((double)config.ball_timeout / 1000.0);
		administrationConfigurator::getInstance().setObstacleTimeToLive((double)config.obstacle_timeout / 1000.0);
		administrationConfigurator::getInstance().setRobotTimeToLive((double)config.teammember_timeout / 1000.0);
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void administratorConfigROS::forceReload()
{
	try
	{
		worldModel::administrationConfig config;

		/* Get standard configuration and update values */
		while(!ros::param::get("worldModelNode/administration/ball_timeout", config.ball_timeout))
		{
			std::cout << "World model administration parameters not loaded yet??" << std::endl;
			sleep(5);
		}

		/* Get parameter values from ROS param-server */
		ros::param::get("worldModelNode/administration/obstacle_timeout", config.obstacle_timeout);
		ros::param::get("worldModelNode/administration/teammember_timeout", config.teammember_timeout);

		administratorConfigROS_cb(config, 0);
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}
