 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * localizationConfigROS.cpp
 *
 *  Created on: Dec 11, 2016
 *      Author: Jan Feitsma
 */

#include "int/adapters/configurators/localizationConfigROS.hpp"

#include "int/configurators/localizationConfigurator.hpp"

#include <ros/ros.h>
#include <tracer.hpp>
#include <FalconsCommon.h> // for loadConfig

#include "cDiagnosticsEvents.hpp"

localizationConfigROS::localizationConfigROS()
{

}

localizationConfigROS::~localizationConfigROS()
/*
 * Chuck Norris once fought superman
 * The loser had to wear his underwear over his pants
 */
{

}

void localizationConfigROS::loadConfigYaml()
{
	try
	{
	    loadConfig("worldModelLocalizationConfig");
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void localizationConfigROS::initializeROS()
{
	try
	{
		loadConfigYaml();

		_srvConfig.reset(new dynamic_reconfigure::Server<worldModel::localizationConfig>(ros::NodeHandle("~/localization")));

		/* Bind the reconfiguration function */
		_f = boost::bind(&localizationConfigROS::localizationConfigROS_cb, this, _1, _2);

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

void localizationConfigROS::localizationConfigROS_cb(worldModel::localizationConfig &config, uint32_t level)
{
	TRACE("callback triggered");
	try
	{
		localizationConfigurator::getInstance().set(localizationConfiguratorFloats::errorRatioRadianToMeter, config.errorRatioRadianToMeter);
		localizationConfigurator::getInstance().set(localizationConfiguratorFloats::visionOwnWeightFactor, config.visionOwnWeightFactor);
		localizationConfigurator::getInstance().set(localizationConfiguratorFloats::trackerScoreAcceptanceThreshold, config.trackerScoreAcceptanceThreshold);
		localizationConfigurator::getInstance().set(localizationConfiguratorFloats::trackerTimeout, config.trackerTimeout);
		localizationConfigurator::getInstance().set(localizationConfiguratorFloats::scoreActivityScale, config.scoreActivityScale);
		localizationConfigurator::getInstance().set(localizationConfiguratorFloats::scoreAgeScale, config.scoreAgeScale);
		localizationConfigurator::getInstance().set(localizationConfiguratorFloats::scoreFreshScale, config.scoreFreshScale);
		localizationConfigurator::getInstance().set(localizationConfiguratorFloats::settlingTime, config.settlingTime);
		localizationConfigurator::getInstance().set(localizationConfiguratorFloats::minimumConfidence, config.minimumConfidence);
		localizationConfigurator::getInstance().set(localizationConfiguratorFloats::speedLimitXY, config.speedLimitXY);
		localizationConfigurator::getInstance().set(localizationConfiguratorFloats::speedLimitPhi, config.speedLimitPhi);
		localizationConfigurator::getInstance().set(localizationConfiguratorFloats::positionLimitX, config.positionLimitX);
		localizationConfigurator::getInstance().set(localizationConfiguratorFloats::positionLimitY, config.positionLimitY);
		localizationConfigurator::getInstance().set(localizationConfiguratorIntegers::visionStabilityLength, config.visionStabilityLength);
		localizationConfigurator::getInstance().traceAll();
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void localizationConfigROS::forceReload()
{
	try
	{
		worldModel::localizationConfig config;

		/* Get standard configuration and update values */
		while(!ros::param::get("worldModelNode/localization/visionOwnWeightFactor", config.visionOwnWeightFactor))
		{
			std::cout << "World model localization parameters not loaded yet??" << std::endl;
			sleep(5);
		}

		/* Get parameter values from ROS param-server */
		ros::param::get("worldModelNode/localization/errorRatioRadianToMeter", config.errorRatioRadianToMeter);
		ros::param::get("worldModelNode/localization/visionOwnWeightFactor", config.visionOwnWeightFactor);
		ros::param::get("worldModelNode/localization/trackerScoreAcceptanceThreshold", config.trackerScoreAcceptanceThreshold);
		ros::param::get("worldModelNode/localization/trackerTimeout", config.trackerTimeout);
		ros::param::get("worldModelNode/localization/scoreActivityScale", config.scoreActivityScale);
		ros::param::get("worldModelNode/localization/scoreAgeScale", config.scoreAgeScale);
		ros::param::get("worldModelNode/localization/scoreFreshScale", config.scoreFreshScale);
		ros::param::get("worldModelNode/localization/settlingTime", config.settlingTime);
		ros::param::get("worldModelNode/localization/minimumConfidence", config.minimumConfidence);
		ros::param::get("worldModelNode/localization/speedLimitXY", config.speedLimitXY);
		ros::param::get("worldModelNode/localization/speedLimitPhi", config.speedLimitPhi);
		ros::param::get("worldModelNode/localization/positionLimitX", config.positionLimitX);
		ros::param::get("worldModelNode/localization/positionLimitY", config.positionLimitY);
		ros::param::get("worldModelNode/localization/visionStabilityLength", config.visionStabilityLength);

		localizationConfigROS_cb(config, 0);
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}
