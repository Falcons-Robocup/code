 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * mainROS.cpp
 *
 *  Created on: Oct 8, 2016
 *      Author: Tim Kouters
 */

//#include <ChuckNorris.hpp>

#include <ros/ros.h>

#include "int/administrators/ballAdministrator.hpp"
#include "int/administrators/obstacleAdministrator.hpp"
#include "int/administrators/robotAdministrator.hpp"
#include "int/adapters/adaptersCollector.hpp"
#include "int/adapters/worldModelInfoUpdater.hpp"
#include "int/adapters/worldModelSyncInfoUpdater.hpp"
#include "int/adapters/ROS/heartBeatAdapterROS.hpp"
#include "int/adapters/ROS/peripheralsAdapterBallPossessionROS.hpp"
#include "int/adapters/ROS/peripheralsAdapterDisplacementROS.hpp"
#include "int/adapters/ROS/peripheralsAdapterRobotStatusROS.hpp"
#include "int/adapters/ROS/visionAdapterBallMeasurementROS.hpp"
#include "int/adapters/ROS/visionAdapterBallPossessionROS.hpp"
#include "int/adapters/ROS/visionAdapterObstacleMeasurementROS.hpp"
#include "int/adapters/ROS/visionAdapterRobotMeasurementROS.hpp"
#include "int/adapters/ROS/wmSyncAdapterBallMeasurementROS.hpp"
#include "int/adapters/ROS/wmSyncAdapterBallPossessionROS.hpp"
#include "int/adapters/ROS/wmSyncAdapterObstacleMeasurementROS.hpp"
#include "int/adapters/ROS/wmSyncAdapterTeamMemberROS.hpp"
#include "int/adapters/ROS/worldModelInfoUpdaterROS.hpp"
#include "int/adapters/ROS/worldModelSyncInfoUpdaterROS.hpp"
#include "int/adapters/ROS/overruleRobotAdapterROS.hpp"
#include "int/adapters/ROS/overruleBallAdapterROS.hpp"
#include "int/adapters/ROS/overruleObstacleAdapterROS.hpp"
#include "int/adapters/ROS/overruleStatusAdapterROS.hpp"
#include "int/adapters/configurators/administratorConfigROS.hpp"
#include "int/adapters/configurators/ballTrackerConfigROS.hpp"
#include "int/adapters/configurators/localizationConfigROS.hpp"
#include "int/adapters/configurators/obstacleTrackerConfigROS.hpp"
#include "ext/WorldModelNames.h"

#include "cDiagnosticsEvents.hpp"

int main(int argc, char **argv)
{
	try
	{
		/* Create administrators */
		robotAdministrator robotAdmin;
		ballAdministrator ballAdmin;
		obstacleAdministrator obstacleAdmin;

		/* Create adapters */
		adaptersCollector adpCollector;
		worldModelInfoUpdater adpWmInfo;
		worldModelSyncInfoUpdater adpWmSyncInfo;

		/* Create ROS adapters */
		heartBeatAdapterROS adpHeartBeatROS;
		peripheralsAdapterBallPossessionROS adpPeriBallPossessionROS;
		peripheralsAdapterDisplacementROS adpPeriDisplacementROS;
		peripheralsAdapterRobotStatusROS adpPeriRobotStatusROS;
		visionAdapterBallMeasurementROS adpVisBallMeasROS;
		visionAdapterBallPossessionROS adpVisBallPossessionROS;
		visionAdapterObstacleMeasurementROS adpVisObstMeasROS;
		visionAdapterRobotMeasurementROS adpVisRobotMeasROS;
		wmSyncAdapterBallMeasurementROS adpWmSyncBallMeasROS;
		wmSyncAdapterBallPossessionROS adpWmSyncBallPossessionROS;
		wmSyncAdapterObstacleMeasurementROS adpWmSyncObstMeasROS;
		wmSyncAdapterTeamMemberROS adpWmSyncTeamMemberROS;
		worldModelInfoUpdaterROS adpWmInfoROS;
		worldModelSyncInfoUpdaterROS adpWmSyncInfoROS;

		/* Overrule ROS adapters */
		overruleRobotAdapterROS adpOverruleRobotROS;
		overruleBallAdapterROS adpOverruleBallROS;
		overruleObstacleAdapterROS adpOverruleObstacleROS;
		overruleStatusAdapterROS adpOverruleStatusROS;

		/* Dynamic reconfigure adapters */
		administratorConfigROS adpAdminConfigROS;
		ballTrackerConfigROS adpBallTrackerConfigROS;
		localizationConfigROS adpLocalizationConfigROS;
		obstacleTrackerConfigROS adpObstacleTrackerConfigROS;

		/* Add admins to adapters */
		adpCollector.setBallAdministrator(&ballAdmin);
		adpCollector.setObstacleAdministrator(&obstacleAdmin);
		adpCollector.setRobotAdministrator(&robotAdmin);
		adpCollector.setWorldModelInfoUpdater(&adpWmInfo);
		adpCollector.setWorldModelSyncInfoUpdater(&adpWmSyncInfo);

		adpWmInfo.setBallAdministrator(&ballAdmin);
		adpWmInfo.setObstacleAdministrator(&obstacleAdmin);
		adpWmInfo.setRobotAdministrator(&robotAdmin);

		adpWmSyncInfo.setBallAdministrator(&ballAdmin);
		adpWmSyncInfo.setObstacleAdministrator(&obstacleAdmin);
		adpWmSyncInfo.setRobotAdministrator(&robotAdmin);

		/* Attach adapters */
		adpWmInfo.setUpdateBallPossession(boost::bind(&worldModelInfoUpdaterROS::setBallPossession, &adpWmInfoROS, _1));
		adpWmInfo.setUpdateFunctionBallLocation(boost::bind(&worldModelInfoUpdaterROS::setBallLocation, &adpWmInfoROS, _1));
		adpWmInfo.setUpdateFunctionObstacles(boost::bind(&worldModelInfoUpdaterROS::setObstacles, &adpWmInfoROS, _1));
		adpWmInfo.setUpdateFunctionRobotLocation(boost::bind(&worldModelInfoUpdaterROS::setRobotLocation, &adpWmInfoROS, _1));
		adpWmInfo.setUpdateFunctionRobotStatus(boost::bind(&worldModelInfoUpdaterROS::setRobotStatus, &adpWmInfoROS, _1));
		adpWmInfo.setUpdateFunctionActiveRobots(boost::bind(&worldModelInfoUpdaterROS::setActiveRobots, &adpWmInfoROS, _1));
		adpWmInfo.setUpdateFunctionSendPacket(boost::bind(&worldModelInfoUpdaterROS::sendPacket, &adpWmInfoROS));
		adpWmInfo.setUpdateFunctionTeamMembers(boost::bind(&worldModelInfoUpdaterROS::setTeamMembers, &adpWmInfoROS, _1));

		adpWmSyncInfo.setUpdateBallPossession(boost::bind(&worldModelSyncInfoUpdaterROS::setBallPossession, &adpWmSyncInfoROS, _1));
		adpWmSyncInfo.setUpdateFunctionBallLocation(boost::bind(&worldModelSyncInfoUpdaterROS::setBallLocations, &adpWmSyncInfoROS, _1));
		adpWmSyncInfo.setUpdateFunctionObstacles(boost::bind(&worldModelSyncInfoUpdaterROS::setObstacles, &adpWmSyncInfoROS, _1));
		adpWmSyncInfo.setUpdateFunctionRobotLocation(boost::bind(&worldModelSyncInfoUpdaterROS::setRobotLocation, &adpWmSyncInfoROS, _1));
		adpWmSyncInfo.setUpdateFunctionRobotStatus(boost::bind(&worldModelSyncInfoUpdaterROS::setRobotStatus, &adpWmSyncInfoROS, _1));
		adpWmSyncInfo.setUpdateFunctionSendPacket(boost::bind(&worldModelSyncInfoUpdaterROS::sendPacket, &adpWmSyncInfoROS));

		adpHeartBeatROS.setUpdateFunction(boost::bind(&adaptersCollector::heartBeatRecalculation, &adpCollector, _1));
		adpPeriBallPossessionROS.setUpdateFunction(boost::bind(&adaptersCollector::updatePeripheralsBallPossession, &adpCollector, _1));
		adpPeriDisplacementROS.setUpdateFunction(boost::bind(&adaptersCollector::updatePeripheralsDisplacement, &adpCollector, _1));
		adpPeriRobotStatusROS.setUpdateFunction(boost::bind(&adaptersCollector::updatePeripheralsRobotStatus, &adpCollector, _1));
		adpVisBallMeasROS.setUpdateFunction(boost::bind(&adaptersCollector::updateVisionBallMeasurement, &adpCollector, _1));
		adpVisBallPossessionROS.setUpdateFunction(boost::bind(&adaptersCollector::updateVisionBallPossession, &adpCollector, _1));
		adpVisObstMeasROS.setUpdateFunction(boost::bind(&adaptersCollector::updateVisionObstacleMeasurement, &adpCollector, _1));
		adpVisRobotMeasROS.setUpdateFunction(boost::bind(&adaptersCollector::updateVisionRobotMeasurement, &adpCollector, _1));
		adpWmSyncBallMeasROS.setUpdateFunction(boost::bind(&adaptersCollector::updateWmSyncBallMeasurement, &adpCollector, _1));
		adpWmSyncBallPossessionROS.setUpdateFunction(boost::bind(&adaptersCollector::updateWmSyncBallPossession, &adpCollector, _1));
		adpWmSyncObstMeasROS.setUpdateFunction(boost::bind(&adaptersCollector::updateWmSyncObstacleMeasurement, &adpCollector, _1));
		adpWmSyncTeamMemberROS.setUpdateFunction(boost::bind(&adaptersCollector::updateWmSyncTeamMember, &adpCollector, _1));

		adpOverruleRobotROS.setUpdateFunction(boost::bind(&adaptersCollector::overruleRobotPosition, &adpCollector, _1));
		adpOverruleBallROS.setUpdateFunction(boost::bind(&adaptersCollector::overruleBallPosition, &adpCollector, _1));
		adpOverruleObstacleROS.setUpdateFunction(boost::bind(&adaptersCollector::overruleObstaclePostions, &adpCollector, _1));
		adpOverruleStatusROS.setUpdateFunction(boost::bind(&adaptersCollector::overruleRobotStatus, &adpCollector, _1));

		/* Init ROS environment */
		ros::init(argc, argv, WorldModelNodeNames::worldmodel_nodename);

		/*
		 * Dynamic reconfigure adapters
		 * -- Load configuration first so that values are in order --
		 */
		adpAdminConfigROS.initializeROS();
		adpBallTrackerConfigROS.initializeROS();
		adpLocalizationConfigROS.initializeROS();
		adpObstacleTrackerConfigROS.initializeROS();

		/* Initialize ROS parts of adapters */
		adpHeartBeatROS.InitializeROS();
		adpPeriBallPossessionROS.InitializeROS();
		adpPeriDisplacementROS.InitializeROS();
		adpPeriRobotStatusROS.InitializeROS();
		adpVisBallMeasROS.InitializeROS();
		adpVisBallPossessionROS.InitializeROS();
		adpVisObstMeasROS.InitializeROS();
		adpVisRobotMeasROS.InitializeROS();
		adpWmSyncBallMeasROS.InitializeROS();
		adpWmSyncBallPossessionROS.InitializeROS();
		adpWmSyncObstMeasROS.InitializeROS();
		adpWmSyncTeamMemberROS.InitializeROS();
		adpWmInfoROS.InitializeROS();
		adpWmSyncInfoROS.InitializeROS();

		/* Overrule adapters */
		adpOverruleRobotROS.InitializeROS();
		adpOverruleBallROS.InitializeROS();
		adpOverruleObstacleROS.InitializeROS();
		adpOverruleStatusROS.InitializeROS();

		/* They see me rollin', they hatin' */
		ros::spin();
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}
