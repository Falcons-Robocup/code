 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
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
	while(!ros::param::get("teamplay_main/shooting/shootPathWidth", config.shootPathWidth))
	{
		std::cout << "Teamplay shooting parameters not loaded yet??" << std::endl;
		sleep(5);
	}
	/* Get parameter values from ROS param-server */
    ros::param::get("teamplay_main/shooting/straightShotThreshold", config.straightShotThreshold);
    ros::param::get("teamplay_main/shooting/aimForCornerThreshold", config.aimForCornerThreshold);
    ros::param::get("teamplay_main/shooting/shotThreshold", config.shotThreshold);

	/* Call configuration file */
	reconfig_cb(config, 0);
}

void cConfigShooting::reconfig_cb(teamplay::teamplayShootingConfig &config, uint32_t level)
{
    teamplay::configurationStore::getConfiguration().setShootPathWidth(config.shootPathWidth);
    teamplay::configurationStore::getConfiguration().setStraightShotThreshold(config.straightShotThreshold);
    teamplay::configurationStore::getConfiguration().setAimForCornerThreshold(config.aimForCornerThreshold);
    teamplay::configurationStore::getConfiguration().setShotThreshold(config.shotThreshold);
}
