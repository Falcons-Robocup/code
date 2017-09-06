 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * main.cpp
 *
 *  Created on: Apr 27, 2014
 *      Author: Tim Kouters
 */

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <boost/any.hpp>
#include <iostream>

#include <compass/compassConfig.h>
#include <compass/Compass.h>
#include <compass/Compass_raw.h>
#include <cCompass.h>
#include <ext/CompassNames.hpp>

using std::cout;
using std::endl;

boost::mutex mtx;
Compass::cCompass *compass = NULL;

void reconfig_cb(compass::compassConfig &config, uint32_t level)
{
	cout << "reconfig received with level " << level << "device "
			<< config.device_name << endl;

	if (compass != NULL)
	{
		delete compass;
		compass = NULL;
	}

	mtx.lock();
	compass = new Compass::cCompass(config.device_name,
	        config.baud_rate, config.homegoal_angle,
	        config.debug_enabled);
	mtx.unlock();
}

bool compass_cb(compass::Compass::Request &req,
		compass::Compass::Response &resp)
{
	float angle = 0.0;
	bool ret_val = true;

	if (compass != NULL)
	{
		mtx.lock();
		compass->get_compass_angle(&angle);
		mtx.unlock();

		resp.theta.data = angle;
	}
	else
	{
		ret_val = false;
	}

	return ret_val;
}

bool compass_raw_cb(compass::Compass_raw::Request &req,
		compass::Compass_raw::Response &resp)
{
	float angle = 0.0;
	bool ret_val = true;

	if (compass != NULL)
	{
		mtx.lock();
		compass->get_raw_compass_angle(&angle);
		mtx.unlock();

		resp.theta.data = angle;
	}
	else
	{
		ret_val = false;
	}

	return ret_val;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, CompassNodeNames::compass_nodename);

	ros::NodeHandle n;
	dynamic_reconfigure::Server<compass::compassConfig> srv;
	dynamic_reconfigure::Server<compass::compassConfig>::CallbackType f;

	/* Load defaults for first construction */
	compass::compassConfig defaultConfig =
			compass::compassConfig::__getDefault__();

	compass = new Compass::cCompass(defaultConfig.device_name,
	        defaultConfig.baud_rate, defaultConfig.homegoal_angle,
	        defaultConfig.debug_enabled);

	/* Bind the reconfiguration function */
	f = boost::bind(&reconfig_cb, _1, _2);
	srv.setCallback(f);

	/* Bind callback function of compass service */
	ros::ServiceServer service_compass = n.advertiseService(CompassInterface::s_get_compass,
			compass_cb);
	ros::ServiceServer service_raw_compass = n.advertiseService(CompassInterface::s_get_compass_raw,
			compass_raw_cb);

    ros::spin();

	return 0;
}

