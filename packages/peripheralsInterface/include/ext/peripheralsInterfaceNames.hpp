 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * peripheralsInterfaceNames.hpp
 *
 *  Created on: Dec 20, 2015
 *      Author: Tim Kouters
 */

#ifndef PERIPHERALSINTERFACENAMES_HPP_
#define PERIPHERALSINTERFACENAMES_HPP_

#include <string>

using std::string;

namespace peripheralsInterfaceNodeNames
{
const static string motors = "peripheralsInterface";
const static string ioBoard = "ioBoard";
}

namespace peripheralsInterfaceServiceNames
{

/* Driving motors */
const static string serviceGetMotionPID = "s_peripheralsInterface_getMotionPID";
const static string serviceSetMotionPID = "s_peripheralsInterface_setMotionPID";
const static string serviceSetRobotSpeed = "s_peripheralsInterface_setRobotSpeed";

/* Ball Handler motors */
const static string getDataBallHandlers = "s_getData_ballHandlers";
const static string setDataBallHandlers = "s_setData_ballHandlers";
const static string getBallhandlerControlMode = "s_get_ballhandler_control_mode";
const static string setBallhandlerControlMode = "s_set_ballhandler_control_mode";

/* Kicker */
const static string setKickPosition = "s_kick_position";
const static string setKickSpeed = "s_kick_speed";
const static string homeKicker = "s_home_kicker";
}

namespace peripheralsInterfaceTopicNames
{
/* Driving motors */
const static string topicMotorRobotSpeed = "t_motor_robot_speed";
const static string topicBallhandlersData = "t_ballhandlers_data";
}

#endif /* PERIPHERALSINTERFACENAMES_HPP_ */
