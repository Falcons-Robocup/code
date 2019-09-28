 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * rtdbKeys.hpp
 *
 *  Created on: Oct 23, 2018
 *      Author: Erik Kouters
 *
 */

#ifndef RTDBKEYS_HPP_
#define RTDBKEYS_HPP_

// !!!!!!!!
// NOTE: If you update this list, please also update the Wiki page: "Software Architecture" and keep things consistent
// !!!!!!!!

#define BALL_CANDIDATES                  "BALL_CANDIDATES"
#define OBSTACLE_CANDIDATES              "OBSTACLE_CANDIDATES"
#define LOCALIZATION_CANDIDATES          "LOCALIZATION_CANDIDATES"
#define VIS_BALL_POSSESSION              "VIS_BALL_POSSESSION"
#define BALL_CANDIDATES_FCS              "BALL_CANDIDATES_FCS"
#define OBSTACLE_CANDIDATES_FCS          "OBSTACLE_CANDIDATES_FCS"
#define ROBOT_STATE                      "ROBOT_STATE"
#define BALLS                            "BALLS"
#define OBSTACLES                        "OBSTACLES"
#define FORBIDDEN_AREAS                  "FORBIDDEN_AREAS"
#define K_ACTION                         "ACTION" // ACTION has naming conflict in Teamplay
#define ACTION_RESULT                    "ACTION_RESULT"
#define MOTION_SETPOINT                  "MOTION_SETPOINT"
#define ROBOT_VELOCITY_SETPOINT          "ROBOT_VELOCITY_SETPOINT"
#define MOTOR_VELOCITY_SETPOINT          "MOTOR_VELOCITY_SETPOINT"
#define BALLHANDLERS_SETPOINT            "BALLHANDLERS_SETPOINT"
#define BALLHANDLERS_MOTOR_SETPOINT      "BALLHANDLERS_MOTOR_SETPOINT"
#define SHOOT_SETPOINT                   "SHOOT_SETPOINT"
#define KICKER_SETPOINT                  "KICKER_SETPOINT"
#define KEEPERFRAME_SETPOINT             "KEEPERFRAME_SETPOINT"
#define MOTOR_FEEDBACK                   "MOTOR_FEEDBACK"
#define ROBOT_DISPLACEMENT_FEEDBACK      "ROBOT_DISPLACEMENT_FEEDBACK"
#define ROBOT_VELOCITY_FEEDBACK          "ROBOT_VELOCITY_FEEDBACK"
#define BALLHANDLERS_FEEDBACK            "BALLHANDLERS_FEEDBACK"
#define BALLHANDLERS_BALL_POSSESSION     "BALLHANDLERS_BALL_POSSESSION"
#define REFBOX_CONFIG                    "REFBOX_CONFIG"
#define MATCH_STATE                      "MATCH_STATE"
#define MATCH_MODE                       "MATCH_MODE"
#define TP_HEARTBEAT                     "TP_HEARTBEAT"
#define INPLAY_FEEDBACK                  "INPLAY_FEEDBACK"
#define EVENT_LIST                       "EVENT_LIST"
#define DIAG_WORLDMODEL_SHARED           "DIAG_WORLDMODEL_SHARED"
#define DIAG_WORLDMODEL_LOCAL            "DIAG_WORLDMODEL_LOCAL"
#define DIAG_TEAMPLAY                    "DIAG_TEAMPLAY"
#define DIAG_PATHPLANNING                "DIAG_PATHPLANNING"
#define DIAG_PERIPHERALSINTERFACE        "DIAG_PERIPHERALSINTERFACE"
#define DIAG_HEALTH_SLOW                 "DIAG_HEALTH_SLOW"
#define DIAG_HEALTH_FAST                 "DIAG_HEALTH_FAST"
#define ROBOT_ROLE                       "ROBOT_ROLE"
#define INTENTION                        "INTENTION"
#define TP_OVERRIDE_STATE                "TP_OVERRIDE_STATE"
#define TP_OVERRIDE_RESULT               "TP_OVERRIDE_RESULT"
#define CONFIG_PATHPLANNING              "CONFIG_PATHPLANNING"
#define CONFIG_WORLDMODELSYNC            "CONFIG_WORLDMODELSYNC"
#define MULTI_CAM_STATISTICS             "MULTI_CAM_STATISTICS"
#define CONFIG_SIMULATION                "CONFIG_SIMULATION"

// !!!!!!!!
// NOTE: If you update this list, please also update the Wiki page: "Software Architecture" and keep things consistent
// http://git.falcons-robocup.nl/falcons/code/wikis/Software-Architecture
// !!!!!!!!

#endif

