 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRosAdapterBallhandlers.hpp
 *
 *  Created on: Aug 3, 2016
 *      Author: Tim Kouters
 */

#ifndef CROSADAPTERBALLHANDLERS_HPP_
#define CROSADAPTERBALLHANDLERS_HPP_

#include <ros/service.h>
#include <ros/node_handle.h>

#include "peripheralsInterface/s_get_ballHandlers_angle.h"
#include "peripheralsInterface/s_set_ballHandlers_angle.h"
#include "peripheralsInterface/s_peripheralsInterface_activatePassBall.h"
#include "peripheralsInterface/s_enable_ballHandlers.h"
#include "peripheralsInterface/s_disable_ballHandlers.h"

#include "rosMsgs/t_bh_pid_params.h"

#include "int/PeripheralsInterfaceData.hpp"

class cRosAdapterBallhandlers {

	public:
	cRosAdapterBallhandlers(PeripheralsInterfaceData& piData);
	~cRosAdapterBallhandlers();

	void initialize();

	void publishPidParameters();

private:
	bool cbGetAngle(peripheralsInterface::s_get_ballHandlers_angle::Request& request, peripheralsInterface::s_get_ballHandlers_angle::Response& response);
	bool cbSetAngle(peripheralsInterface::s_set_ballHandlers_angle::Request& request, peripheralsInterface::s_set_ballHandlers_angle::Response& response);
	bool cbActivatePassBall(peripheralsInterface::s_peripheralsInterface_activatePassBall::Request& request, peripheralsInterface::s_peripheralsInterface_activatePassBall::Response& response);
	bool cbEnableBallhandlers(peripheralsInterface::s_enable_ballHandlers::Request&, peripheralsInterface::s_enable_ballHandlers::Response& response);
	bool cbDisableBallhandlers(peripheralsInterface::s_disable_ballHandlers::Request&, peripheralsInterface::s_disable_ballHandlers::Response& response);

	PeripheralsInterfaceData &_piData;

	ros::NodeHandle _n;

	ros::ServiceServer _srvGetAngle;
	ros::ServiceServer _srvSetAngle;
	ros::ServiceServer _srvActivatePassBall;
	ros::ServiceServer _srvEnableBallhandlers;
	ros::ServiceServer _srvDisableBallhandlers;

	ros::Publisher  _tBhPidParameters;

};

#endif /* CROSADAPTERBALLHANDLERS_HPP_ */
