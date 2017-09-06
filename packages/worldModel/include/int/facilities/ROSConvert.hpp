 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ROSConvert.hpp
 *
 *  Created on: Oct 8, 2016
 *      Author: Tim Kouters
 */

#ifndef ROSCONVERT_HPP_
#define ROSCONVERT_HPP_

#include <stdint.h>

#include "rosMsgs/s_camera_type.h"
#include "rosMsgs/s_robot_status.h"
#include "rosMsgs/BallPossession.h"

#include "int/types/robot/robotStatusType.hpp"
#include "int/types/ball/ballPossessionType.hpp"
#include "int/types/cameraType.hpp"

cameraType convertCameraType(const rosMsgs::s_camera_type camType);
rosMsgs::s_camera_type convertCameraType(const cameraType camType);

robotStatusType convertRobotStatus(const rosMsgs::s_robot_status statusType);
rosMsgs::s_robot_status convertRobotStatus(const robotStatusType statusType);

bool convertIsClaimedByTeam(const rosMsgs::BallPossession claimType);

rosMsgs::BallPossession convertBallPossession(ballPossessionClass_t possession);
ballPossessionClass_t convertBallPossession(const rosMsgs::BallPossession ballPossessionROS, const float x, const float y);


#endif /* ROSCONVERT_HPP_ */
