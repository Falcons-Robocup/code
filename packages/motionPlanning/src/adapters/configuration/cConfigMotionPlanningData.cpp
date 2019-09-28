 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cConfigMotionPlanningData.cpp
 *
 *  Created on: April 19 2018
 */

#include "int/adapters/configuration/cConfigMotionPlanningData.hpp"
#include "int/stores/configurationStore.hpp"

#include <ros/param.h>
#include <pwd.h> // for getpwuid()

#include "FalconsCommon.h"
#include "tracing.hpp"


cConfigMotionPlanningData::cConfigMotionPlanningData()
{
	loadConfigYaml();
}

cConfigMotionPlanningData::~cConfigMotionPlanningData()
{

}

void cConfigMotionPlanningData::loadConfigYaml()
{
	motionPlanning::MotionPlanningNodeConfig config;

	/* Execute loading of ros params */
	struct passwd *pw = getpwuid(getuid());
	std::string configFileCmd("rosparam load ");
	configFileCmd.append(pw->pw_dir);
	configFileCmd.append("/falcons/code/config/motionPlanning.yaml");
	int dummy_val = system(configFileCmd.c_str());
	TRACE("ros parameters loaded, returncode=%d", dummy_val);
	dummy_val = 0;

	/* Bind the reconfiguration function */
	_srv.reset(new dynamic_reconfigure::Server<motionPlanning::MotionPlanningNodeConfig>());
	_f = boost::bind(&cConfigMotionPlanningData::reconfig_cb, this, _1, _2);
	_srv->setCallback(_f);

	/* Get standard configuration and update values */
   while(!ros::param::get("MotionPlanningNode/getBall_obstacleThreshold", config.getBall_obstacleThreshold))
	{
		std::cout << "Motion Planning data parameters not loaded yet??" << std::endl;
		sleep(5);
	}
	/* Get parameter values from ROS param-server */
   ros::param::get("MotionPlanningNode/getBall_ballSpeedThreshold", config.getBall_ballSpeedThreshold );
   ros::param::get("MotionPlanningNode/getBall_ballSpeedScaling", config.getBall_ballSpeedScaling );

   ros::param::get("MotionPlanningNode/interceptBall_obstacleThreshold", config.interceptBall_obstacleThreshold );
   ros::param::get("MotionPlanningNode/interceptBall_ballSpeedThreshold", config.interceptBall_ballSpeedThreshold );
   ros::param::get("MotionPlanningNode/interceptBall_ballSpeedScaling", config.interceptBall_ballSpeedScaling );

   ros::param::get("MotionPlanningNode/keeperMove_YMaxOffset", config.keeperMove_YMaxOffset );
   ros::param::get("MotionPlanningNode/keeperMove_XGoalpostOffset", config.keeperMove_XGoalpostOffset );
   ros::param::get("MotionPlanningNode/keeperMove_XYThreshold", config.keeperMove_XYThreshold );

   ros::param::get("MotionPlanningNode/moveToTarget_XYThreshold", config.moveToTarget_XYThreshold );
   ros::param::get("MotionPlanningNode/moveToTarget_PhiThreshold", config.moveToTarget_PhiThreshold );

   ros::param::get("MotionPlanningNode/passToTarget_accuracy", config.passToTarget_accuracy );
   ros::param::get("MotionPlanningNode/passToTarget_timeout", config.passToTarget_timeout );

   ros::param::get("MotionPlanningNode/shootAtTarget_aimSettleTime", config.shootAtTarget_aimSettleTime );
   ros::param::get("MotionPlanningNode/shootAtTarget_accuracy", config.shootAtTarget_accuracy );
   ros::param::get("MotionPlanningNode/shootAtTarget_timeout", config.shootAtTarget_timeout );
   ros::param::get("MotionPlanningNode/shootAtTarget_coarseAngle", config.shootAtTarget_coarseAngle );
   ros::param::get("MotionPlanningNode/shootAtTarget_disableBhDelay", config.shootAtTarget_disableBhDelay );
   ros::param::get("MotionPlanningNode/shootAtTarget_sleepAfterShoot", config.shootAtTarget_sleepAfterShoot );

   ros::param::get("MotionPlanningNode/turnAwayFromOpponent_PhiThreshold", config.turnAwayFromOpponent_PhiThreshold );

	/* Call configuration file */
	reconfig_cb(config, 0);
}


void cConfigMotionPlanningData::reconfig_cb(motionPlanning::MotionPlanningNodeConfig &config, uint32_t level)
{
    motionPlanning::configurationStore::getConfiguration().setGetBall_ObstacleThreshold((float) config.getBall_obstacleThreshold);
    motionPlanning::configurationStore::getConfiguration().setGetBall_BallSpeedThreshold((float) config.getBall_ballSpeedThreshold);
    motionPlanning::configurationStore::getConfiguration().setGetBall_BallSpeedScaling((float) config.getBall_ballSpeedScaling);

    motionPlanning::configurationStore::getConfiguration().setInterceptBall_ObstacleThreshold((float) config.interceptBall_obstacleThreshold);
    motionPlanning::configurationStore::getConfiguration().setInterceptBall_BallSpeedThreshold((float) config.interceptBall_ballSpeedThreshold);
    motionPlanning::configurationStore::getConfiguration().setInterceptBall_BallSpeedScaling((float) config.interceptBall_ballSpeedScaling);

    motionPlanning::configurationStore::getConfiguration().setKeeperMove_YMaxOffset((float) config.keeperMove_YMaxOffset);
    motionPlanning::configurationStore::getConfiguration().setKeeperMove_XGoalpostOffset((float) config.keeperMove_XGoalpostOffset);
    motionPlanning::configurationStore::getConfiguration().setKeeperMove_XYThreshold((float) config.keeperMove_XYThreshold);

    motionPlanning::configurationStore::getConfiguration().setMoveToTarget_XYThreshold((float) config.moveToTarget_XYThreshold);
    motionPlanning::configurationStore::getConfiguration().setMoveToTarget_PhiThreshold((float) config.moveToTarget_PhiThreshold);

    motionPlanning::configurationStore::getConfiguration().setPassToTarget_Accuracy((float) config.passToTarget_accuracy);
    motionPlanning::configurationStore::getConfiguration().setPassToTarget_Timeout((float) config.passToTarget_timeout);

    motionPlanning::configurationStore::getConfiguration().setShootAtTarget_AimSettleTime((float) config.shootAtTarget_aimSettleTime);
    motionPlanning::configurationStore::getConfiguration().setShootAtTarget_Accuracy((float) config.shootAtTarget_accuracy);
    motionPlanning::configurationStore::getConfiguration().setShootAtTarget_Timeout((float) config.shootAtTarget_timeout);
    motionPlanning::configurationStore::getConfiguration().setShootAtTarget_CoarseAngle((float) config.shootAtTarget_coarseAngle);
    motionPlanning::configurationStore::getConfiguration().setShootAtTarget_DisableBHDelay((float) config.shootAtTarget_disableBhDelay);
    motionPlanning::configurationStore::getConfiguration().setShootAtTarget_SleepAfterShoot((float) config.shootAtTarget_sleepAfterShoot);

    motionPlanning::configurationStore::getConfiguration().setTurnAwayFromOpponent_PhiThreshold((float) config.turnAwayFromOpponent_PhiThreshold);
}
