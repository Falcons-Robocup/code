 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cConfigStrategy.cpp
 *
 *  Created on: July 24, 2017
 *      Author: Ivo Matthijssen
 */

#include "int/adapters/configuration/cConfigStrategy.hpp"

#include <ros/param.h>
#include <pwd.h> // for getpwuid()

#include "FalconsCommon.h"

#include "int/stores/configurationStore.hpp"

cConfigStrategy::cConfigStrategy()
{
	loadConfigYaml();
}

cConfigStrategy::~cConfigStrategy()
{

}

void cConfigStrategy::loadConfigYaml()
{
	teamplay::teamplayStrategyConfig config;

	/* Execute loading of ros params */
	struct passwd *pw = getpwuid(getuid());
	std::string configFileCmd("rosparam load ");
	configFileCmd.append(pw->pw_dir);
	configFileCmd.append("/falcons/code/config/teamplayStrategy.yaml");
	int dummy_val = system(configFileCmd.c_str());
	TRACE("ros parameters loaded, returncode=%d", dummy_val);
	dummy_val = 0;

	/* Bind the reconfiguration function */
	_srv.reset(new dynamic_reconfigure::Server<teamplay::teamplayStrategyConfig>(ros::NodeHandle("~/strategy")));
	_f = boost::bind(&cConfigStrategy::reconfig_cb, this, _1, _2);
	_srv->setCallback(_f);

	/* Get standard configuration and update values */
	while(!ros::param::get("teamplay_main/strategy/defendingStrategy", config.defendingStrategy))
	{
		std::cout << "Teamplay rules parameters not loaded yet??" << std::endl;
		sleep(5);
	}

	/* Call configuration file */
	reconfig_cb(config, 0);
}

void cConfigStrategy::reconfig_cb(teamplay::teamplayStrategyConfig &config, uint32_t level)
{
    teamplay::configurationStore::getConfiguration().setDefendingStrategy(config.defendingStrategy);
}
