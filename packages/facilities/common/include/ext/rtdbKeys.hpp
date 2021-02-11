// Copyright 2018-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
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

#define BALL_CANDIDATES                                 "BALL_CANDIDATES"
#define OBSTACLE_CANDIDATES                             "OBSTACLE_CANDIDATES"
#define LOCALIZATION_CANDIDATES                         "LOCALIZATION_CANDIDATES"
#define VIS_BALL_POSSESSION                             "VIS_BALL_POSSESSION"
#define BALL_CANDIDATES_FCS                             "BALL_CANDIDATES_FCS"
#define OBSTACLE_CANDIDATES_FCS                         "OBSTACLE_CANDIDATES_FCS"
#define ROBOT_STATE                                     "ROBOT_STATE"
#define BALLS                                           "BALLS"
#define OBSTACLES                                       "OBSTACLES"
#define FORBIDDEN_AREAS                                 "FORBIDDEN_AREAS"
#define K_ACTION                                        "ACTION" // ACTION has naming conflict in Teamplay
#define ACTION_RESULT                                   "ACTION_RESULT"
#define MOTION_SETPOINT                                 "MOTION_SETPOINT"
#define ROBOT_POSVEL_SETPOINT                           "ROBOT_POSVEL_SETPOINT"
#define ROBOT_VELOCITY_SETPOINT                         "ROBOT_VELOCITY_SETPOINT"
#define MOTOR_VELOCITY_SETPOINT                         "MOTOR_VELOCITY_SETPOINT"
#define BALLHANDLERS_SETPOINT                           "BALLHANDLERS_SETPOINT"
#define BALLHANDLERS_MOTOR_SETPOINT                     "BALLHANDLERS_MOTOR_SETPOINT"
#define SHOOT_SETPOINT                                  "SHOOT_SETPOINT"
#define KICKER_SETPOINT                                 "KICKER_SETPOINT"
#define KEEPERFRAME_SETPOINT                            "KEEPERFRAME_SETPOINT"
#define MOTOR_FEEDBACK                                  "MOTOR_FEEDBACK"
#define ROBOT_DISPLACEMENT_FEEDBACK                     "ROBOT_DISPLACEMENT_FEEDBACK"
#define ROBOT_VELOCITY_FEEDBACK                         "ROBOT_VELOCITY_FEEDBACK"
#define BALLHANDLERS_FEEDBACK                           "BALLHANDLERS_FEEDBACK"
#define BALLHANDLERS_BALL_POSSESSION                    "BALLHANDLERS_BALL_POSSESSION"
#define REFBOX_CONFIG                                   "REFBOX_CONFIG"
#define REFBOX_OVERRIDE                                 "REFBOX_OVERRIDE"
#define MATCH_STATE                                     "MATCH_STATE"
#define MATCH_MODE                                      "MATCH_MODE"
#define TP_HEARTBEAT                                    "TP_HEARTBEAT"
#define HEARTBEAT_COACH                                 "HEARTBEAT_COACH"
#define INPLAY_FEEDBACK                                 "INPLAY_FEEDBACK"
#define EVENT_LIST                                      "EVENT_LIST"
#define DIAG_WORLDMODEL_SHARED                          "DIAG_WORLDMODEL_SHARED"
#define DIAG_WORLDMODEL_LOCAL                           "DIAG_WORLDMODEL_LOCAL"
#define DIAG_TEAMPLAY                                   "DIAG_TEAMPLAY"
#define DIAG_PATHPLANNING                               "DIAG_PATHPLANNING"
#define DIAG_VELOCITYCONTROL                            "DIAG_VELOCITYCONTROL"
#define DIAG_BALLHANDLING                               "DIAG_BALLHANDLING"
#define DIAG_PERIPHERALSINTERFACE                       "DIAG_PERIPHERALSINTERFACE"
#define DIAG_HEALTH_SLOW                                "DIAG_HEALTH_SLOW"
#define DIAG_HEALTH_FAST                                "DIAG_HEALTH_FAST"
#define ROBOT_ROLE                                      "ROBOT_ROLE"
#define INTENTION                                       "INTENTION"
#define TP_OVERRIDE_STATE                               "TP_OVERRIDE_STATE"
#define TP_OVERRIDE_RESULT                              "TP_OVERRIDE_RESULT"
#define CONFIG_WORLDMODEL                               "CONFIG_WORLDMODEL"
#define CONFIG_HEARTBEATCOACH                           "CONFIG_HEARTBEATCOACH"
#define CONFIG_PATHPLANNING                             "CONFIG_PATHPLANNING"
#define CONFIG_VELOCITYCONTROL                          "CONFIG_VELOCITYCONTROL"
#define CONFIG_BALLHANDLING                             "CONFIG_BALLHANDLING"
#define CONFIG_MOTIONPLANNING                           "CONFIG_MOTIONPLANNING"
#define CONFIG_SHOOTPLANNING                            "CONFIG_SHOOTPLANNING"
#define CONFIG_PERIPHERALSINTERFACE_MOTORS              "CONFIG_PI_MOTORS"
#define CONFIG_PERIPHERALSINTERFACE_BALLHANDLERS        "CONFIG_PI_BALLHANDLERS"
#define CONFIG_PERIPHERALSINTERFACE_KICKER              "CONFIG_PI_KICKER"
#define CONFIG_WORLDMODELSYNC                           "CONFIG_WORLDMODELSYNC"
#define MULTI_CAM_STATISTICS                            "MULTI_CAM_STATISTICS"
#define CONFIG_SIMULATION                               "CONFIG_SIMULATION"
#define CONFIG_TEAMPLAY                                 "CONFIG_TEAMPLAY"
#define SIMULATION_SCENE                                "SIMULATION_SCENE"
#define SIMULATION_TIME                                 "SIMULATION_TIME"
#define SIMULATION_TICK                                 "SIMULATION_TICK"
#define DIAG_TRUE_BALL                                  "DIAG_TRUE_BALL"
#define VISION_OBJECTS                                  "VISION_OBJECTS"

// !!!!!!!!
// NOTE: If you update this list, please also update the Wiki page: "Software Architecture" and keep things consistent
// http://git.falcons-robocup.nl/falcons/code/wikis/Software-Architecture
// !!!!!!!!

#endif


