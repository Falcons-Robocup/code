 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * peripheralInterfaceNames.hpp
 *
 *  Created on: Dec 20, 2015
 *      Author: Tim Kouters
 */

#ifndef PERIPHERALINTERFACENAMES_HPP_
#define PERIPHERALINTERFACENAMES_HPP_

#include <string>

using std::string;

namespace peripheralInterfaceNodeNames
{
const static string motors = "peripheralsInterface";
const static string compass = "compass";
const static string ioBoard = "ioBoard";
}

namespace peripheralInterfaceServiceNames
{
/* Compass */
const static string s_get_compass = "s_get_compass";
const static string s_get_raw_compass = "s_get_raw_compass";

/* Driving motors */
const static string serviceGetMotionPID = "s_peripheralsInterface_getMotionPID";
const static string serviceSetMotionPID = "s_peripheralsInterface_setMotionPID";
const static string serviceSetRobotSpeed = "s_peripheralsInterface_setRobotSpeed";

/* Ball Handler motors */
const static string serviceActivatePassBall = "s_peripheralsInterface_activatePassBall";
const static string getBallHandlersAngle = "s_get_ballHandlers_angle";
const static string setBallHandlersAngle = "s_set_ballHandlers_angle";
const static string enableBallHandlers = "s_enable_ballHandlers";
const static string disableBallHandlers = "s_disable_ballHandlers";

/* Kicker */
const static string setKickPosition = "s_kick_position";
const static string setKickSpeed = "s_kick_speed";
const static string homeKicker = "s_home_kicker";
}

namespace peripheralInterfaceTopicNames
{
	/* Compass */
	const static string topicCompass = "t_compass";

	/* Driving motors */
	const static string topicPIDParameters = "t_motor_pid_params";
	const static string topicMotorRobotSpeed = "t_motor_robot_speed";
	const static std::string topicBHPIDParameters = "t_bh_pid_params";
}

#endif /* PERIPHERALINTERFACENAMES_HPP_ */
