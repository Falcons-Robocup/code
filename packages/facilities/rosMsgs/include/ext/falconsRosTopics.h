 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * falconsRosTopics.h
 *
 *  Created on: May 8, 2016
 *      Author: Diana Koenraadt
 */

#ifndef FALCONROSSTOPICS_H_
#define FALCONROSSTOPICS_H_

namespace falconsRosTopicsInterface
{
    // World model:
    const static std::string team_worldmodel = "/teamA/g_worldmodel_team"; 
    const static std::string team_worldmodel_old = "/teamA/g_worldmodel"; // legacy wmV1

    // Diagnostics:
    // Note: Uses boost format-string
    const static std::string robot_worldmodel = "/teamA/robot%1%/g_diag_worldmodel";
    const static std::string robot_wm_loc = "/teamA/robot%1%/g_diag_wm_loc";
    const static std::string robot_wm_ball = "/teamA/robot%1%/g_diag_wm_ball";
    const static std::string robot_wm_obst = "/teamA/robot%1%/g_diag_wm_obstacles";
    const static std::string robot_wm_top = "/teamA/robot%1%/g_diag_wm_top";
    const static std::string robot_control = "/teamA/robot%1%/g_diag_control";
    const static std::string robot_vision = "/teamA/robot%1%/g_diag_vision";
    const static std::string robot_vision_v2 = "/teamA/robot%1%/g_diag_vision_v2";
    const static std::string robot_health_slow = "/teamA/robot%1%/g_diag_health_slow";
    const static std::string robot_health_mid = "/teamA/robot%1%/g_diag_health_mid";
    const static std::string robot_health_fast = "/teamA/robot%1%/g_diag_health_fast";
    const static std::string robot_pathplanning = "/teamA/robot%1%/g_diag_pathpl";
    const static std::string robot_teamplay = "/teamA/robot%1%/g_diag_teamplay";
    const static std::string robot_error = "/teamA/robot%1%/g_diag_error";
    const static std::string robot_info = "/teamA/robot%1%/g_diag_info";
    const static std::string robot_halmw = "/teamA/robot%1%/g_diag_halmw"; // TODO use full name in C++. What does MW stand for?
    const static std::string robot_compass = "/teamA/robot%1%/g_diag_compass";
    const static std::string robot_active = "/teamA/robot%1%/g_diag_active";
    const static std::string robot_frontvision = "/teamA/robot%1%/g_diag_frontvision";
    const static std::string robot_refbox = "/teamA/robot%1%/g_diag_refbox";
    const static std::string robot_ana_comm = "/teamA/robot%1%/g_ana_comm";

    // Analytics:
    const static std::string analytics_event = "/teamA/g_ana_event"; 
    const static std::string analytics_online = "/teamA/g_ana_online"; 
    const static std::string analytics_matchstate = "/teamA/g_ana_matchstate"; 
}

#endif /* FALCONROSSTOPICS_H_ */
