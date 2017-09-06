 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * configuratorWorldModelPacketROS.cpp
 *
 *  Created on: Oct 11, 2016
 *      Author: Tim Kouters
 */

#include "int/adapters/configuratorWorldModelPacketROS.hpp"

#include "int/configurators/configuratorWorldModelPacket.hpp"

#include <pwd.h> // for getpwuid()
#include "FalconsCommon.h"
#include "cDiagnosticsEvents.hpp"

configuratorWorldModelPacketROS::configuratorWorldModelPacketROS()
{

}

configuratorWorldModelPacketROS::~configuratorWorldModelPacketROS()
{
	TRACE(">");

	TRACE("<");
}


void configuratorWorldModelPacketROS::loadDefaultParameters()
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
    			configFileCmd.append("/falcons/code/config/WorldModelSyncSim_teamB.yaml");
		    }
		    else
		    {
    			configFileCmd.append("/falcons/code/config/WorldModelSyncSim.yaml");
		    }
		}
		else
		{
			configFileCmd.append("/falcons/code/config/WorldModelSync.yaml");
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

void configuratorWorldModelPacketROS::initializeROS()
{
	try
	{
		/* Bind the reconfiguration function */
		_srvDynConfig_cb = boost::bind(&configuratorWorldModelPacketROS::reconfig_cb, this, _1, _2);
		_srvDynConfig.setCallback(_srvDynConfig_cb);
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void configuratorWorldModelPacketROS::reconfig_cb(worldModelSync::worldmodelsyncConfig &config, uint32_t level)
{
	try
	{
		/*
		 * Read new values from ROS param server
		 */
		std::cout << "Reading WorldModel Sync parameters from ROS parameter server" << std::endl;
        while (!ros::param::get("WorldModelSync/multicastAddress", config.multicastAddress))
        {
            std::cout << "WorldModel Sync parameters not loaded yet??" << std::endl;
            sleep(5);
        }
        ros::param::get("WorldModelSync/multicastPort", config.multicastPort);
        // JFEI hardcode +10 port offset for now to allow teamB next to teamA for parallel development
        // details in #245 and http://timmel.no-ip.biz:8000/wiki/MSLNetworkSetup#SoftwareIPaddressandportusage
        if (getTeamChar() == 'B')
    	{
        	config.multicastPort += 10;
    	}
        ros::param::get("WorldModelSync/allowLoop", config.allowLoop);
        ros::param::get("WorldModelSync/multicastHop", config.multicastHop);
        ros::param::get("WorldModelSync/groupID", config.groupID);

        /*
         * Put new values in configurator
         */
        configuratorWorldModelPacket::getInstance().setMulticastAddress(config.multicastAddress);
        configuratorWorldModelPacket::getInstance().setPortNumber(config.multicastPort);
        configuratorWorldModelPacket::getInstance().setLoopIsEnabled(config.allowLoop);
        configuratorWorldModelPacket::getInstance().setMaxHops(config.multicastHop);
        configuratorWorldModelPacket::getInstance().setGroupID(config.groupID);

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

