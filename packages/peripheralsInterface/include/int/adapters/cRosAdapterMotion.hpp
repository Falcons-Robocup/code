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
 *  Created on: Okt 1, 2015
 *      Author: Edwin Schreuder
 */

#ifndef CROSADAPTERMOTION_HPP_
#define CROSADAPTERMOTION_HPP_

#include <ros/ros.h>

#include "peripheralsInterface/s_peripheralsInterface_getMotionPID.h"
#include "peripheralsInterface/s_peripheralsInterface_setMotionPID.h"
#include "peripheralsInterface/s_peripheralsInterface_setRobotSpeed.h"

#include "rosMsgs/t_robotspeed.h"
#include "rosMsgs/t_motor_pid_params.h"

#include "int/PeripheralsInterfaceTypes.hpp"
#include "int/PeripheralsInterfaceData.hpp"

class cRosAdapterMotion {

public:
	cRosAdapterMotion(PeripheralsInterfaceData& piData);
	~cRosAdapterMotion();

	void initialize();

	void publishRobotSpeed();
	void publishPidParameters();

private:
	bool cbGetMotionPID(peripheralsInterface::s_peripheralsInterface_getMotionPID::Request& request, peripheralsInterface::s_peripheralsInterface_getMotionPID::Response& response);
	bool cbSetMotionPID(peripheralsInterface::s_peripheralsInterface_setMotionPID::Request& request, peripheralsInterface::s_peripheralsInterface_setMotionPID::Response& response);
	bool cbSetRobotSpeed(peripheralsInterface::s_peripheralsInterface_setRobotSpeed::Request& request, peripheralsInterface::s_peripheralsInterface_setRobotSpeed::Response& response);
	void cbRobotSpeed(const rosMsgs::t_robotspeed::ConstPtr& msg);

	PeripheralsInterfaceData &_piData;

	ros::NodeHandle _n;

	ros::Publisher _tMotorPidParameters;
	ros::Publisher _tMotorRobotSpeed;

	ros::ServiceServer _srvGetMotionPID;
	ros::ServiceServer _srvSetMotionPID;
	ros::ServiceServer _srvGetRobotSpeed;
	ros::ServiceServer _srvSetRobotSpeed;

	ros::Subscriber _subTarget;
};

#endif /* CROSADAPTERMOTION_HPP_ */
