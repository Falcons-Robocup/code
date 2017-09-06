 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cConfigShooting.cpp
 *
 *  Created on: May 19, 2017
 *      Author: Coen Tempelaars
 */

#include "int/adapters/configuration/cConfigShooting.hpp"

#include <ros/param.h>

#include "FalconsCommon.h"

#include "int/stores/configurationStore.hpp"

cConfigShooting::cConfigShooting()
{
	loadConfigYaml();
}

cConfigShooting::~cConfigShooting()
{

}

void cConfigShooting::loadConfigYaml()
{
	teamplay::teamplayShootingConfig config;

	/* Execute loading of ros params */
	loadConfig("teamplayShooting");

	/* Bind the reconfiguration function */
	_srv.reset(new dynamic_reconfigure::Server<teamplay::teamplayShootingConfig>(ros::NodeHandle("~/shooting")));
	_f = boost::bind(&cConfigShooting::reconfig_cb, this, _1, _2);
	_srv->setCallback(_f);

	/* Get standard configuration and update values */
	while(!ros::param::get("teamplay_main/shooting/shootAtGoalPower", config.shootAtGoalPower))
	{
		std::cout << "Teamplay shooting parameters not loaded yet??" << std::endl;
		sleep(5);
	}
	/* Get parameter values from ROS param-server */
	ros::param::get("teamplay_main/shooting/shortPassPower", config.shortPassPower);
	ros::param::get("teamplay_main/shooting/shortToMediumPassPower", config.shortToMediumPassPower);
	ros::param::get("teamplay_main/shooting/mediumPassPower", config.mediumPassPower);
	ros::param::get("teamplay_main/shooting/mediumToLongPassPower", config.mediumToLongPassPower);
	ros::param::get("teamplay_main/shooting/longPassPower", config.longPassPower);

    ros::param::get("teamplay_main/shooting/maxDistanceForShortPass", config.maxDistanceForShortPass);
    ros::param::get("teamplay_main/shooting/maxDistanceForShortToMediumPass", config.maxDistanceForShortToMediumPass);
    ros::param::get("teamplay_main/shooting/maxDistanceForMediumPass", config.maxDistanceForMediumPass);
    ros::param::get("teamplay_main/shooting/maxDistanceForMediumToLongPass", config.maxDistanceForMediumToLongPass);

    ros::param::get("teamplay_main/shooting/shootTimer", config.shootTimer);
    ros::param::get("teamplay_main/shooting/shootTimerAngleThreshold", config.shootTimerAngleThreshold);
    ros::param::get("teamplay_main/shooting/shootPathWidth", config.shootPathWidth);
    ros::param::get("teamplay_main/shooting/settleTimeAfterShooting", config.settleTimeAfterShooting);

	/* Call configuration file */
	reconfig_cb(config, 0);
}

void cConfigShooting::reconfig_cb(teamplay::teamplayShootingConfig &config, uint32_t level)
{
    teamplay::passPowers _passPowers;
    _passPowers[teamplay::passDistance::SHORT] = config.shortPassPower;
    _passPowers[teamplay::passDistance::SHORT_TO_MEDIUM] = config.shortToMediumPassPower;
    _passPowers[teamplay::passDistance::MEDIUM] = config.mediumPassPower;
    _passPowers[teamplay::passDistance::MEDIUM_TO_LONG] = config.mediumToLongPassPower;
    _passPowers[teamplay::passDistance::LONG] = config.longPassPower;

    teamplay::passRanges _passRanges;
    _passRanges[teamplay::passDistance::SHORT] = config.maxDistanceForShortPass;
    _passRanges[teamplay::passDistance::SHORT_TO_MEDIUM] = config.maxDistanceForShortToMediumPass;
    _passRanges[teamplay::passDistance::MEDIUM] = config.maxDistanceForMediumPass;
    _passRanges[teamplay::passDistance::MEDIUM_TO_LONG] = config.maxDistanceForMediumToLongPass;

    teamplay::configurationStore::getConfiguration().setShootAtGoalPower(config.shootAtGoalPower);
    teamplay::configurationStore::getConfiguration().setPassPowers(_passPowers);
    teamplay::configurationStore::getConfiguration().setPassRanges(_passRanges);
    teamplay::configurationStore::getConfiguration().setShootTimer(config.shootTimer);
    teamplay::configurationStore::getConfiguration().setShootTimerAngleThreshold(config.shootTimerAngleThreshold);
    teamplay::configurationStore::getConfiguration().setShootPathWidth(config.shootPathWidth);
    teamplay::configurationStore::getConfiguration().setSettleTimeAfterShooting(config.settleTimeAfterShooting);
}
