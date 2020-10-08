 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * rtdbStructs.hpp
 *
 *  Created on: Nov 1, 2018
 *      Author: Jan Feitsma
 *
 */

#ifndef RTDBSTRUCTS_HPP_
#define RTDBSTRUCTS_HPP_

// sharedTypes
#include "ballMeasurement.hpp"
#include "obstacleMeasurement.hpp"
#include "robotLocalizationMeasurement.hpp"
#include "robotState.hpp"
#include "ballResult.hpp"
#include "obstacleResult.hpp"
#include "forbiddenArea.hpp"
#include "action.hpp"
#include "actionResult.hpp"
#include "motionSetpoint.hpp"
#include "robotVelocity.hpp"
#include "motorsTargetVelocity.hpp"
#include "ballHandlersSetpoint.hpp"
#include "ballHandlersMotorSetpoint.hpp"
#include "shootSetpoint.hpp"
#include "kickerSetpoint.hpp"
#include "keeperFrameSetpoint.hpp"
#include "motorsFeedback.hpp"
#include "robotDisplacement.hpp"
#include "ballHandlersMotorData.hpp"
#include "refboxConfig.hpp"
#include "refboxOverride.hpp"
#include "matchState.hpp"
#include "robotStatusEnum.hpp"
#include "event.hpp"
#include "diagWorldModel.hpp"
#include "diagTeamplay.hpp"
#include "diagPathPlanning.hpp"
#include "DiagBallHandling.hpp"
#include "diagPeripheralsInterface.hpp"
#include "diagHealthSlow.hpp"
#include "diagHealthFast.hpp"
#include "intention.hpp"
#include "tpOverrideState.hpp"
#include "tpOverrideResult.hpp"
#include "ConfigWorldModel.hpp"
#include "ConfigHeartBeatCoach.hpp"
#include "ConfigPathPlanning.hpp"
#include "ConfigBallHandling.hpp"
#include "ConfigMotionPlanning.hpp"
#include "ConfigShootPlanning.hpp"
#include "ConfigPeripheralsInterfaceMotors.hpp"
#include "ConfigPeripheralsInterfaceBallHandlers.hpp"
#include "ConfigPeripheralsInterfaceKicker.hpp"
#include "configWorldModelSync.hpp"
#include "multiCamStatistics.hpp"
#include "configSimulation.hpp"
#include "configTeamplay.hpp"
#include "SimulationScene.hpp"

// generated in sharedTypes
#include "generated_enum2str.hpp"

// !!!!!!!!
// NOTE: If you update this list, please also update the Wiki page: "Software Architecture" and keep things consistent
// !!!!!!!!

typedef std::vector<ballMeasurement>              T_BALL_CANDIDATES;
typedef std::vector<obstacleMeasurement>          T_OBSTACLE_CANDIDATES;
typedef std::vector<robotLocalizationMeasurement> T_LOCALIZATION_CANDIDATES;
typedef bool                                      T_VIS_BALL_POSSESSION;
typedef std::vector<ballMeasurement>              T_BALL_CANDIDATES_FCS;
typedef std::vector<obstacleMeasurement>          T_OBSTACLE_CANDIDATES_FCS;
typedef robotState                                T_ROBOT_STATE;
typedef std::vector<ballResult>                   T_BALLS;
typedef std::vector<obstacleResult>               T_OBSTACLES;
typedef std::vector<forbiddenArea>                T_FORBIDDEN_AREAS;
typedef action                                    T_ACTION;
typedef actionResult                              T_ACTION_RESULT;
typedef motionSetpoint                            T_MOTION_SETPOINT;
typedef robotVelocity                             T_ROBOT_VELOCITY_SETPOINT;
typedef motorsTargetVelocity                      T_MOTOR_VELOCITY_SETPOINT;
typedef ballHandlersSetpoint                      T_BALLHANDLERS_SETPOINT;
typedef ballHandlersMotorSetpoint                 T_BALLHANDLERS_MOTOR_SETPOINT;
typedef shootSetpoint                             T_SHOOT_SETPOINT;
typedef kickerSetpoint                            T_KICKER_SETPOINT;
typedef keeperFrameSetpoint                       T_KEEPERFRAME_SETPOINT;
typedef motorsFeedback                            T_MOTOR_FEEDBACK;
typedef robotDisplacement                         T_ROBOT_DISPLACEMENT_FEEDBACK;
typedef robotVelocity                             T_ROBOT_VELOCITY_FEEDBACK;
typedef ballHandlersMotorData                     T_BALLHANDLERS_FEEDBACK;
typedef bool                                      T_BALLHANDLERS_BALL_POSSESSION;
typedef refboxConfig                              T_REFBOX_CONFIG;
typedef refboxOverride                            T_REFBOX_OVERRIDE;
typedef matchState                                T_MATCH_STATE;
typedef bool                                      T_MATCH_MODE;
typedef int                                       T_TP_HEARTBEAT;
typedef int                                       T_HEARTBEAT_COACH;
typedef robotStatusEnum                           T_INPLAY_FEEDBACK;
typedef std::vector<event>                        T_EVENT_LIST;
typedef diagWorldModelShared                      T_DIAG_WORLDMODEL_SHARED;
typedef diagWorldModelLocal                       T_DIAG_WORLDMODEL_LOCAL;
typedef diagTeamplay                              T_DIAG_TEAMPLAY;
typedef diagPathPlanning                          T_DIAG_PATHPLANNING;
typedef DiagBallHandling                          T_DIAG_BALLHANDLING;
typedef diagPeripheralsInterface                  T_DIAG_PERIPHERALSINTERFACE;
typedef diagHealthSlow                            T_DIAG_HEALTH_SLOW;
typedef diagHealthFast                            T_DIAG_HEALTH_FAST;
typedef std::string                               T_ROBOT_ROLE;
typedef intention                                 T_INTENTION;
typedef tpOverrideState                           T_TP_OVERRIDE_STATE;
typedef tpOverrideResult                          T_TP_OVERRIDE_RESULT;
typedef ConfigWorldModel                          T_CONFIG_WORLDMODEL;
typedef ConfigHeartBeatCoach                      T_CONFIG_HEARTBEATCOACH;
typedef ConfigPathPlanning                        T_CONFIG_PATHPLANNING;
typedef ConfigBallHandling                        T_CONFIG_BALLHANDLING;
typedef ConfigMotionPlanning                      T_CONFIG_MOTIONPLANNING;
typedef ConfigShootPlanning                       T_CONFIG_SHOOTPLANNING;
typedef ConfigPeripheralsInterfaceMotors          T_CONFIG_PERIPHERALSINTERFACE_MOTORS;
typedef ConfigPeripheralsInterfaceBallHandlers    T_CONFIG_PERIPHERALSINTERFACE_BALLHANDLERS;
typedef ConfigPeripheralsInterfaceKicker          T_CONFIG_PERIPHERALSINTERFACE_KICKER;
typedef configWorldModelSync                      T_CONFIG_WORLDMODELSYNC;
typedef multiCamStatistics                        T_MULTI_CAM_STATISTICS;
typedef configSimulation                          T_CONFIG_SIMULATION;
typedef configTeamplay                            T_CONFIG_TEAMPLAY;
typedef SimulationScene                           T_SIMULATION_SCENE;
typedef double                                    T_SIMULATION_TIME;
typedef int                                       T_SIMULATION_TICK;

// !!!!!!!!
// NOTE: If you update this list, please also update the Wiki page: "Software Architecture" and keep things consistent
// !!!!!!!!

#endif


