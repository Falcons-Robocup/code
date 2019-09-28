 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * WorldModelNodeNames.h
 *
 *  Created on: May 26, 2014
 *      Author: Tim Kouters
 */

#ifndef WORLDMODELNODENAMES_H_
#define WORLDMODELNODENAMES_H_

#include <string>

namespace WorldModelNodeNames
{

const static std::string worldmodel_nodename = "worldModelNode";
}

namespace WorldModelInterface
{

const static std::string s_set_own_location = "s_set_own_location";
const static std::string s_set_member_location = "s_set_member_location";
const static std::string s_set_own_obstacle_location = "s_set_own_obstacle_location";
const static std::string s_set_remote_obstacle_location = "s_set_remote_obstacle_location";
const static std::string s_set_own_ball_location = "s_set_own_ball_location";
const static std::string s_set_remote_ball_location = "s_set_remote_ball_location";
const static std::string s_set_own_front_camera_ball_location = "s_set_own_front_camera_ball_location";
const static std::string s_set_remote_ball_possession = "s_set_remote_ball_possession";
const static std::string s_claim_own_ball_possession = "s_claim_own_ball_possession";
const static std::string s_release_own_ball_possession = "s_release_own_ball_possession";
const static std::string s_set_own_encoder_displacement = "s_set_own_encoder_displacement";
const static std::string s_set_own_robot_status = "s_set_own_robot_status";
const static std::string s_set_own_camera_ball_possession = "s_set_own_camera_ball_possession";
const static std::string t_wmInfo = "t_wmInfo";
const static std::string t_wMSyncInfo = "t_wMSyncInfo";

const static std::string s_set_overrule_own_location = "s_set_overrule_own_location";
const static std::string s_set_overrule_ball_location = "s_set_overrule_ball_location";
const static std::string s_set_overrule_obstacle_location = "s_set_overrule_obstacle_location";
const static std::string s_set_overrule_robot_status = "s_set_overrule_robot_status";
}

#endif /* WORLDMODELNODENAMES_H_ */
