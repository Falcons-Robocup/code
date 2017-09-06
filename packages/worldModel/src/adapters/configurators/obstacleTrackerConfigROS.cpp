 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * obstacleTrackerConfigROS.cpp
 *
 *  Created on: Nov 22, 2016
 *      Author: Tim Kouters
 */

#include "int/adapters/configurators/obstacleTrackerConfigROS.hpp"

#include "int/configurators/obstacleTrackerConfigurator.hpp"

#include <ros/ros.h>
#include <tracer.hpp>
#include <FalconsCommon.h> // for loadConfig

#include "cDiagnosticsEvents.hpp"

obstacleTrackerConfigROS::obstacleTrackerConfigROS()
{

}

obstacleTrackerConfigROS::~obstacleTrackerConfigROS()
/*
 * Chuck Norris once robbed himself and doubled his profit
 */
{

}

void obstacleTrackerConfigROS::loadConfigYaml()
{
	try
	{
	    loadConfig("worldModelObstacleTrackerConfig");
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void obstacleTrackerConfigROS::initializeROS()
{
	try
	{
		loadConfigYaml();

		_srvConfig.reset(new dynamic_reconfigure::Server<worldModel::obstacletrackerConfig>(ros::NodeHandle("~/obstacleTracker")));

		/* Bind the reconfiguration function */
		_f = boost::bind(&obstacleTrackerConfigROS::obstacleTrackerConfigROS_cb, this, _1, _2);

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

void obstacleTrackerConfigROS::obstacleTrackerConfigROS_cb(worldModel::obstacletrackerConfig &config, uint32_t level)
{
	TRACE("callback triggered");
	try
	{
		obstacleTrackerConfigurator::getInstance().set(obstacleTrackerConfiguratorFloats::discriminatorLimitX, config.discriminatorLimitX);
		obstacleTrackerConfigurator::getInstance().set(obstacleTrackerConfiguratorFloats::discriminatorLimitY, config.discriminatorLimitY);
		obstacleTrackerConfigurator::getInstance().set(obstacleTrackerConfiguratorFloats::extrapolationTimeout, config.extrapolationTimeout);
		obstacleTrackerConfigurator::getInstance().set(obstacleTrackerConfiguratorFloats::trackerTimeout, config.trackerTimeout);
		obstacleTrackerConfigurator::getInstance().set(obstacleTrackerConfiguratorFloats::trackerXYTolerance, config.trackerXYTolerance);
		obstacleTrackerConfigurator::getInstance().set(obstacleTrackerConfiguratorFloats::filterXYownTolerance, config.filterXYownTolerance);
		obstacleTrackerConfigurator::getInstance().set(obstacleTrackerConfiguratorFloats::filterXYmemberTolerance, config.filterXYmemberTolerance);
		obstacleTrackerConfigurator::getInstance().set(obstacleTrackerConfiguratorFloats::outlierNSigma, config.outlierNSigma);
		obstacleTrackerConfigurator::getInstance().set(obstacleTrackerConfiguratorFloats::outlierIterFraction, config.outlierIterFraction);
		obstacleTrackerConfigurator::getInstance().set(obstacleTrackerConfiguratorFloats::speedMinSize, config.speedMinSize);
		obstacleTrackerConfigurator::getInstance().set(obstacleTrackerConfiguratorFloats::speedMaxSize, config.speedMaxSize);
		obstacleTrackerConfigurator::getInstance().set(obstacleTrackerConfiguratorFloats::speedResidualThreshold, config.speedResidualThreshold);
		
		obstacleTrackerConfigurator::getInstance().set(obstacleTrackerConfiguratorIntegers::outlierMaxIter, config.outlierMaxIter);
		obstacleTrackerConfigurator::getInstance().set(obstacleTrackerConfiguratorIntegers::speedFitOrder, config.speedFitOrder);
		
		obstacleTrackerConfigurator::getInstance().traceAll();
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void obstacleTrackerConfigROS::forceReload()
{
	try
	{
		worldModel::obstacletrackerConfig config;

		/* Get standard configuration and update values */
		while(!ros::param::get("worldModelNode/obstacleTracker/discriminatorLimitX", config.discriminatorLimitX))
		{
			std::cout << "World model obstacle tracker parameters not loaded yet??" << std::endl;
			sleep(5);
		}

		/* Get parameter values from ROS param-server */
		ros::param::get("worldModelNode/obstacleTracker/discriminatorLimitY", config.discriminatorLimitY);
		ros::param::get("worldModelNode/obstacleTracker/trackerTimeout", config.trackerTimeout);
		ros::param::get("worldModelNode/obstacleTracker/extrapolationTimeout", config.extrapolationTimeout);
		ros::param::get("worldModelNode/obstacleTracker/trackerXYTolerance", config.trackerXYTolerance);
		ros::param::get("worldModelNode/obstacleTracker/filterXYownTolerance", config.filterXYownTolerance);
		ros::param::get("worldModelNode/obstacleTracker/filterXYmemberTolerance", config.filterXYmemberTolerance);
		ros::param::get("worldModelNode/obstacleTracker/outlierNSigma", config.outlierNSigma);
		ros::param::get("worldModelNode/obstacleTracker/outlierMaxIter", config.outlierMaxIter);
		ros::param::get("worldModelNode/obstacleTracker/speedFitOrder", config.speedFitOrder);
		ros::param::get("worldModelNode/obstacleTracker/speedMinSize", config.speedMinSize);
		ros::param::get("worldModelNode/obstacleTracker/speedMaxSize", config.speedMaxSize);
		ros::param::get("worldModelNode/obstacleTracker/speedResidualThreshold", config.speedResidualThreshold);
		ros::param::get("worldModelNode/obstacleTracker/outlierIterFraction", config.outlierIterFraction);
		obstacleTrackerConfigROS_cb(config, 0);
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}
