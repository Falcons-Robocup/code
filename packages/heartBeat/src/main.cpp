 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * main.cpp
 *
 *  Created on: Nov 10, 2015
 *      Author: Tim Kouters
 */

#include <stdexcept>

#include <ros/ros.h>

#include "ext/heartBeatNames.hpp"
#include "int/config/cHeartBeatConfig.hpp"
#include "int/adapters/cHeartBeatConfigRosAdapter.hpp"
#include "int/adapters/cBeatMessageRosAdapter.hpp"
#include "cDiagnostics.hpp"

#include "FalconsCommon.h"

using std::exception;

int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, heartBeatNodeNames::heartBeatNodeName);
        //TRACE_INFO("heartbeat starting"); // must be after ros::init

        /*
         * Create object for heartbeat generation
         */
        cBeatMessageRosAdapter heartBeat;

        /*
         * Fetch configuration from yaml file
         */
        cHeartBeatConfigRosAdapter configRos;

        /*
         * Create loop for heart beat generation
         */
        ros::Rate loopRate(cHeartBeatConfig::getInstance().getUpdateFrequency());
        while(ros::ok())
        {
            //TRACE("heartbeat LATENCY start");
        	heartBeat.publishHeartBeat();
        	ros::spinOnce();
        	loopRate.sleep();

        	/*
        	 * Update loop rate in case frequency is updated
        	 */
        	loopRate = ros::Rate(cHeartBeatConfig::getInstance().getUpdateFrequency());
        }
    }
    catch (exception &e)
    {
    	throw e;
    }

}
