 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * configuratorMixedTeamPacketROS.cpp
 *
 *  Created on: Oct 11, 2016
 *      Author: Tim Kouters
 */

#include "int/adapters/configuratorMixedTeamPacketROS.hpp"

#include "int/configurators/configuratorMixedTeamPacket.hpp"

#include <pwd.h> // for getpwuid()
#include "FalconsCommon.h"
#include "tracing.hpp"
#include "cDiagnostics.hpp"

configuratorMixedTeamPacketROS::configuratorMixedTeamPacketROS()
{

}

configuratorMixedTeamPacketROS::~configuratorMixedTeamPacketROS()
{

}


void configuratorMixedTeamPacketROS::loadDefaultParameters()
{
	try
	{
		struct passwd *pw = getpwuid(getuid());
		std::string configFileCmd("rosparam load ");
		configFileCmd.append(pw->pw_dir);
		if (isSimulatedEnvironment())
		{
		    // for team A, use WorldModelSyncSim.yaml
		    // for team B, use WorldModelSyncSim_teamB.yaml
		    if (getTeamChar() == 'B')
		    {
    			configFileCmd.append("/falcons/code/config/WorldModelSyncSim_teamBMixedTeam.yaml");
		    }
		    else
		    {
    			configFileCmd.append("/falcons/code/config/WorldModelSyncSimMixedTeam.yaml");
		    }
		}
		else
		{
			configFileCmd.append("/falcons/code/config/WorldModelSyncMixedTeam.yaml");
		}
		TRACE("loading configFile: %s", configFileCmd.c_str());
		int dummy_val = system(configFileCmd.c_str());
		dummy_val++;
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void configuratorMixedTeamPacketROS::initializeROS()
{
	try
	{
		/* Bind the reconfiguration function */
		_srvConfig.reset(new dynamic_reconfigure::Server<worldModelSync::mixedteamConfig>(ros::NodeHandle("~/mixedTeam")));
		_srvDynConfig_cb = boost::bind(&configuratorMixedTeamPacketROS::reconfig_cb, this, _1, _2);
		_srvConfig->setCallback(_srvDynConfig_cb);
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void configuratorMixedTeamPacketROS::reconfig_cb(worldModelSync::mixedteamConfig &config, uint32_t level)
{
	try
	{
		/*
		 * Read new values from ROS param server
		 */
		std::cout << "Reading WorldModel Sync parameters from ROS parameter server" << std::endl;
        while (!ros::param::get("WorldModelSync/mixedTeam/multicastMixedAddress", config.multicastMixedAddress))
        {
            std::cout << "WorldModel Sync parameters not loaded yet??" << std::endl;
            sleep(5);
        }
        ros::param::get("WorldModelSync/mixedTeam/multicastMixedPort", config.multicastMixedPort);
        ros::param::get("WorldModelSync/mixedTeam/allowLoop", config.allowLoop);
        ros::param::get("WorldModelSync/mixedTeam/multicastHop", config.multicastHop);
        ros::param::get("WorldModelSync/mixedTeam/groupID", config.groupID);
        ros::param::get("WorldModelSync/mixedTeam/sendMixedTeam", config.sendMixedTeam);

        /*
         * Put new values in configurator
         */
        configuratorMixedTeamPacket::getInstance().setMulticastAddress(config.multicastMixedAddress);
        configuratorMixedTeamPacket::getInstance().setPortNumber(config.multicastMixedPort);
        configuratorMixedTeamPacket::getInstance().setLoopIsEnabled(config.allowLoop);
        configuratorMixedTeamPacket::getInstance().setMaxHops(config.multicastHop);
        configuratorMixedTeamPacket::getInstance().setGroupID(config.groupID);
        configuratorMixedTeamPacket::getInstance().setMixedTeamEnabled(config.sendMixedTeam);

        /*
         * Notify new configuration is loaded
         */
        for(auto it = _fncNotifyNewConfigurations.begin(); it != _fncNotifyNewConfigurations.end(); it++)
        {
        	(*it)();
        }
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}
