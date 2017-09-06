 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * globalConfig.cpp
 *
 *  Created on: Mar 28, 2016
 *      Author: Jan Feitsma
 */



#include <ros/ros.h>
#include <FalconsCommon.h>
#include <dynamic_reconfigure/server.h>
#include "configManager/configManagerConfig.h"
#include "rosMsgs/t_diag_globalconfig.h"

ros::Publisher pub;


void reconfig_cb(configManager::configManagerConfig &config, uint32_t level)
{
    TRACE("reconfig");
    rosMsgs::t_diag_globalconfig msg;
    msg.config = systemStdout("configShow");
    pub.publish(msg);
}


int main(int argc, char **argv)
{
    // init
    ros::init(argc, argv, "globalConfig");
    
    // define output topic
    std::string topic = "g_diag_globalconfig";
    ros::NodeHandle nh;
    pub = nh.advertise<rosMsgs::t_diag_globalconfig>(topic, 20.0);
    
    // setup configuration
    TRACE("setup configuration");
    dynamic_reconfigure::Server<configManager::configManagerConfig> srv;
    dynamic_reconfigure::Server<configManager::configManagerConfig>::CallbackType f;
	f = boost::bind(&reconfig_cb, _1, _2);
	srv.setCallback(f);

    // spin
    TRACE("spinning");
    ros::spin();
}

