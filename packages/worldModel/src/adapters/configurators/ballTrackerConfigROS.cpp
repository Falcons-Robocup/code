 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ballTrackerConfigROS.cpp
 *
 *  Created on: Nov 22, 2016
 *      Author: Tim Kouters
 */

#include "int/adapters/configurators/ballTrackerConfigROS.hpp"

#include "int/configurators/ballTrackerConfigurator.hpp"

#include <ros/ros.h>
#include "tracing.hpp"
#include <FalconsCommon.h> // for loadConfig

#include "cDiagnostics.hpp"

ballTrackerConfigROS::ballTrackerConfigROS()
{

}

ballTrackerConfigROS::~ballTrackerConfigROS()
/*
 * Chuck Norris once fought superman
 * The loser had to wear his underwear over his pants
 */
{

}

void ballTrackerConfigROS::loadConfigYaml()
{
	try
	{
	    loadConfig("worldModelBallTrackerConfig");
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void ballTrackerConfigROS::initializeROS()
{
	try
	{
		loadConfigYaml();

		_srvConfig.reset(new dynamic_reconfigure::Server<worldModel::balltrackerConfig>(ros::NodeHandle("~/ballTracker")));

		/* Bind the reconfiguration function */
		_f = boost::bind(&ballTrackerConfigROS::ballTrackerConfigROS_cb, this, _1, _2);

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

void ballTrackerConfigROS::ballTrackerConfigROS_cb(worldModel::balltrackerConfig &config, uint32_t level)
{
	TRACE("callback triggered");
	try
	{
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorBool::confidenceOmniPref, config.confidenceOmniPref);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorBool::solverCoreSpeed, config.solverCoreSpeed);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorBool::useOwnHighVision, config.useOwnHighVision);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorBool::useFriendlyHighVision, config.useFriendlyHighVision);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorBool::shareHighVision, config.shareHighVision);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorBool::blackListDefault, config.blackListDefault);

		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorIntegers::confidenceMeasLim, config.confidenceMeasLim);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorIntegers::confidenceNumCams, config.confidenceNumCams);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorIntegers::maxMaybeBalls, config.maxMaybeBalls);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorIntegers::numberOfBallsWarningThreshold, config.numberOfBallsWarningThreshold);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorIntegers::solverBounceMinMeasurements, config.solverBounceMinMeasurements);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorIntegers::solverCoreMinVmeas, config.solverCoreMinVmeas);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorIntegers::measPerOrder, config.measPerOrder);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorIntegers::outlierMaxIter, config.outlierMaxIter);

		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::confidenceAgeLim, config.confidenceAgeLim);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::confidenceFitLim1, config.confidenceFitLim1);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::confidenceFitLim2, config.confidenceFitLim2);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::confidenceFreshLim, config.confidenceFreshLim);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::confidenceGoodLimit, config.confidenceGoodLimit);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::confidenceMaybeLimit, config.confidenceMaybeLimit);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::confidenceVLim1, config.confidenceVLim1);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::confidenceVLim2, config.confidenceVLim2);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::confidenceZLim1, config.confidenceZLim1);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::confidenceZLim2, config.confidenceZLim2);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::solverBounceAge, config.solverBounceAge);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::solverBounceDt, config.solverBounceDt);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::solverMinDv, config.solverMinDv);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::solverTrackerTimeout, config.solverTrackerTimeout);
        ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::solverTrackerBuffer, config.solverTrackerBuffer);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::solverTrackerConeTolerance, config.solverTrackerConeTolerance);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::groupingDt, config.groupingDt);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::outlierNSigma, config.outlierNSigma);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::outlierIterFraction, config.outlierIterFraction);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::friendlyMeasurementsDistance, config.friendlyMeasurementsDistance);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::blackListThresholdZ, config.blackListThresholdZ);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::blackListFloatingDuration, config.blackListFloatingDuration);
		ballTrackerConfigurator::getInstance().set(ballTrackerConfiguratorFloats::blackListGroundDuration, config.blackListGroundDuration);

		ballTrackerConfigurator::getInstance().traceAll();

	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void ballTrackerConfigROS::forceReload()
{
	try
	{
		worldModel::balltrackerConfig config;

		/* Get standard configuration and update values */
		while(!ros::param::get("worldModelNode/ballTracker/confidenceAgeLim", config.confidenceAgeLim))
		{
			std::cout << "World model ball tracker parameters not loaded yet??" << std::endl;
			sleep(5);
		}

		/* Get parameter values from ROS param-server */
		ros::param::get("worldModelNode/ballTracker/confidenceFitLim1", config.confidenceFitLim1);
		ros::param::get("worldModelNode/ballTracker/confidenceFitLim2", config.confidenceFitLim2);
		ros::param::get("worldModelNode/ballTracker/confidenceFreshLim", config.confidenceFreshLim);
		ros::param::get("worldModelNode/ballTracker/confidenceGoodLimit", config.confidenceGoodLimit);
		ros::param::get("worldModelNode/ballTracker/confidenceMaybeLimit", config.confidenceMaybeLimit);
		ros::param::get("worldModelNode/ballTracker/confidenceMeasLim", config.confidenceMeasLim);
		ros::param::get("worldModelNode/ballTracker/confidenceNumCams", config.confidenceNumCams);
		ros::param::get("worldModelNode/ballTracker/confidenceOmniPref", config.confidenceOmniPref);
		ros::param::get("worldModelNode/ballTracker/confidenceVLim1", config.confidenceVLim1);
		ros::param::get("worldModelNode/ballTracker/confidenceVLim2", config.confidenceVLim2);
		ros::param::get("worldModelNode/ballTracker/confidenceZLim1", config.confidenceZLim1);
		ros::param::get("worldModelNode/ballTracker/confidenceZLim2", config.confidenceZLim2);
		ros::param::get("worldModelNode/ballTracker/maxMaybeBalls", config.maxMaybeBalls);
		ros::param::get("worldModelNode/ballTracker/numberOfBallsWarningThreshold", config.numberOfBallsWarningThreshold);
		ros::param::get("worldModelNode/ballTracker/solverBounceAge", config.solverBounceAge);
		ros::param::get("worldModelNode/ballTracker/solverBounceDt", config.solverBounceDt);
		ros::param::get("worldModelNode/ballTracker/solverBounceMinMeasurements", config.solverBounceMinMeasurements);
		ros::param::get("worldModelNode/ballTracker/solverCoreMinVdt", config.solverCoreMinVdt);
		ros::param::get("worldModelNode/ballTracker/solverCoreMinVmeas", config.solverCoreMinVmeas);
		ros::param::get("worldModelNode/ballTracker/solverCoreSpeed", config.solverCoreSpeed);
		ros::param::get("worldModelNode/ballTracker/solverCoreWeight", config.solverCoreWeight);
		ros::param::get("worldModelNode/ballTracker/solverMinDv", config.solverMinDv);
		ros::param::get("worldModelNode/ballTracker/solverTrackerTimeout", config.solverTrackerTimeout);
		ros::param::get("worldModelNode/ballTracker/solverTrackerBuffer", config.solverTrackerBuffer);
		ros::param::get("worldModelNode/ballTracker/solverTrackerConeTolerance", config.solverTrackerConeTolerance);
		ros::param::get("worldModelNode/ballTracker/measPerOrder", config.measPerOrder);
		ros::param::get("worldModelNode/ballTracker/groupingDt", config.groupingDt);
		ros::param::get("worldModelNode/ballTracker/outlierNSigma", config.outlierNSigma);
		ros::param::get("worldModelNode/ballTracker/outlierIterFraction", config.outlierIterFraction);		
		ros::param::get("worldModelNode/ballTracker/outlierMaxIter", config.outlierMaxIter);
		ros::param::get("worldModelNode/ballTracker/useOwnHighVision", config.useOwnHighVision);
		ros::param::get("worldModelNode/ballTracker/useFriendlyHighVision", config.useFriendlyHighVision);
		ros::param::get("worldModelNode/ballTracker/shareHighVision", config.shareHighVision);
		ros::param::get("worldModelNode/ballTracker/friendlyMeasurementsDistance", config.friendlyMeasurementsDistance);
		ros::param::get("worldModelNode/ballTracker/blackListDefault", config.blackListDefault);
		ros::param::get("worldModelNode/ballTracker/blackListThresholdZ", config.blackListThresholdZ);
		ros::param::get("worldModelNode/ballTracker/blackListFloatingDuration", config.blackListFloatingDuration);
		ros::param::get("worldModelNode/ballTracker/blackListGroundDuration", config.blackListGroundDuration);

		ballTrackerConfigROS_cb(config, 0);
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}
