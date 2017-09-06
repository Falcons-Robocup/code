 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRosAdapterKicker.hpp
 *
 *  Created on: Apr 24, 2017
 *      Author: Edwin Schreuder
 */

#ifndef INCLUDE_INT_ADAPTERS_CROSADAPTERKICKER_HPP_
#define INCLUDE_INT_ADAPTERS_CROSADAPTERKICKER_HPP_

#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>

#include "peripheralsInterface/s_peripheralsInterface_setKickPosition.h"
#include "peripheralsInterface/s_peripheralsInterface_setKickSpeed.h"
#include "peripheralsInterface/s_homeKicker.h"

#include "peripheralsInterface/kickerConfig.h"

#include "int/Kicker.hpp"

class cRosAdapterKicker {

public:
	cRosAdapterKicker(Kicker& kicker);
	~cRosAdapterKicker();

	void initialize();

private:
	bool cbSetKickPosition(peripheralsInterface::s_peripheralsInterface_setKickPosition::Request& request, peripheralsInterface::s_peripheralsInterface_setKickPosition::Response& response);
	bool cbSetKickSpeed(peripheralsInterface::s_peripheralsInterface_setKickSpeed::Request& request, peripheralsInterface::s_peripheralsInterface_setKickSpeed::Response& response);
	bool cbHomeKicker(peripheralsInterface::s_homeKicker::Request& request, peripheralsInterface::s_homeKicker::Response& response);

	void cbConfig(peripheralsInterface::kickerConfig &config, uint32_t level);

	Kicker &kicker;

	ros::NodeHandle nodeHandle;
	ros::ServiceServer setKickPositionService;
	ros::ServiceServer setKickSpeedService;
	ros::ServiceServer homeKickerService;

	dynamic_reconfigure::Server<peripheralsInterface::kickerConfig> configServer;
};

#endif /* INCLUDE_INT_ADAPTERS_CROSADAPTERKICKER_HPP_ */
