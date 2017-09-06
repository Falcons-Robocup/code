 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cHeartBeatConfigRosAdapter.cpp
 *
 *  Created on: Nov 10, 2015
 *      Author: Tim Kouters
 */

#include "int/adapters/cHeartBeatConfigRosAdapter.hpp"

#include <boost/any.hpp>
#include <dynamic_reconfigure/server.h>
#include <stdexcept>
#include <string>
#include <pwd.h> // for getpwuid()

#include "FalconsCommon.h"
#include "int/config/cHeartBeatConfig.hpp"

using std::cout;
using std::endl;
using std::exception;
using std::string;


/*
 * Class implementation
 */
cHeartBeatConfigRosAdapter::cHeartBeatConfigRosAdapter()
{
	loadConfiguration();
}

cHeartBeatConfigRosAdapter::~cHeartBeatConfigRosAdapter()
{

}

void cHeartBeatConfigRosAdapter::loadConfiguration()
{

	dynamic_reconfigure::Server<heartBeat::heartBeatConfig> srv;
	dynamic_reconfigure::Server<heartBeat::heartBeatConfig>::CallbackType f;

	/* Reconfig run */
    loadConfig("heartBeat");

	/* Bind the reconfiguration function */
	f = boost::bind(&reconfigHeartBeat_cb, _1, _2, this);
	srv.setCallback(f);
}

void cHeartBeatConfigRosAdapter::reconfigHeartBeat_cb(heartBeat::heartBeatConfig &config, uint32_t level, cHeartBeatConfigRosAdapter *classRef)
{
	try
	{
		cout << "Reading heartBeat parameters from ROS parameter server" << endl;
		while (!ros::param::get("heartBeat/updateFrequency", config.updateFrequency))
		{
			cout << "heartBeat parameters not loaded yet??" << endl;
			sleep(5);
		}

		cHeartBeatConfig::getInstance().setUpdateFrequency(config.updateFrequency);

	} catch (exception &e)
	{
		cout << "Error occurred at reconfiguring heartBeat" << e.what() << endl;
	}
}
