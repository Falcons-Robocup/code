 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cConfigRules.cpp
 *
 *  Created on: May 24, 2016
 *      Author: robocup
 */

#include "int/adapters/configuration/cConfigRules.hpp"

#include <ros/param.h>
#include <pwd.h> // for getpwuid()

#include "FalconsCommon.h"
#include "tracing.hpp"

#include "int/stores/configurationStore.hpp"

cConfigRules::cConfigRules()
{
	loadConfigYaml();
}

cConfigRules::~cConfigRules()
{

}

void cConfigRules::loadConfigYaml()
{
	teamplay::teamplayRulesConfig config;

	/* Execute loading of ros params */
	struct passwd *pw = getpwuid(getuid());
	std::string configFileCmd("rosparam load ");
	configFileCmd.append(pw->pw_dir);
	configFileCmd.append("/falcons/code/config/teamplayRules.yaml");
	int dummy_val = system(configFileCmd.c_str());
	TRACE("rules parameters loaded, returncode=%d", dummy_val);
	dummy_val = 0;

	/* Bind the reconfiguration function */
	_srv.reset(new dynamic_reconfigure::Server<teamplay::teamplayRulesConfig>(ros::NodeHandle("~/rules")));
	_f = boost::bind(&cConfigRules::reconfig_cb, this, _1, _2);
	_srv->setCallback(_f);

	/* Get standard configuration and update values */
	while(!ros::param::get("teamplay_main/rules/setpieceExecuteTimeout", config.setpieceExecuteTimeout))
	{
		std::cout << "Teamplay rules parameters not loaded yet??" << std::endl;
		sleep(5);
	}
	/* Get parameter values from ROS param-server */
	ros::param::get("teamplay_main/rules/penaltyExecuteTimeout", config.penaltyExecuteTimeout);
	ros::param::get("teamplay_main/rules/minKickDistanceKicked", config.minKickDistanceKicked);
	ros::param::get("teamplay_main/rules/minPenaltyDistanceKicked", config.minPenaltyDistanceKicked);
	ros::param::get("teamplay_main/rules/minOwnKickoffDistanceKicked", config.minOwnKickoffDistanceKicked);
	ros::param::get("teamplay_main/rules/stimulatePassingEnabled", config.stimulatePassingEnabled);

	/* Call configuration file */
	reconfig_cb(config, 0);
}

void cConfigRules::reconfig_cb(teamplay::teamplayRulesConfig &config, uint32_t level)
{
    teamplay::configurationStore::getConfiguration().setSetPieceExecuteTimeout((float) config.setpieceExecuteTimeout / 1000.0);
    teamplay::configurationStore::getConfiguration().setPenaltyExecuteTimeout((float) config.penaltyExecuteTimeout / 1000.0);

    teamplay::configurationStore::getConfiguration().setMinKickDistanceKickedMeters(config.minKickDistanceKicked);
    teamplay::configurationStore::getConfiguration().setMinPenaltyDistanceKickedMeters(config.minPenaltyDistanceKicked);
    teamplay::configurationStore::getConfiguration().setMinOwnKickoffDistanceKickedMeters(config.minOwnKickoffDistanceKicked);

    teamplay::configurationStore::getConfiguration().setRuleStimulatePassing(config.stimulatePassingEnabled);
}
