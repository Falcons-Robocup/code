// Copyright 2015-2018 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
