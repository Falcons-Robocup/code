// Copyright 2021-2022 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
#include "robotPosVel.hpp"
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
#include "diagVelocityControl.hpp"
#include "DiagBallHandling.hpp"
#include "diagPeripheralsInterface.hpp"
#include "diagHealthSlow.hpp"
#include "diagHealthFast.hpp"
#include "diagTrueBall.hpp"
#include "intention.hpp"
#include "tpOverrideState.hpp"
#include "tpOverrideResult.hpp"
#include "ConfigWorldModel.hpp"
#include "ConfigHeartBeatCoach.hpp"
#include "ConfigExecution.hpp"
#include "ConfigTeamplay.hpp"
#include "ConfigPathPlanning.hpp"
#include "ConfigVelocityControl.hpp"
#include "ConfigVelocityTransform.hpp"
#include "ConfigBallHandling.hpp"
#include "ConfigMotionPlanning.hpp"
#include "ConfigShootPlanning.hpp"
#include "ConfigPeripheralsInterfaceMotors.hpp"
#include "ConfigPeripheralsInterfaceBallHandlers.hpp"
#include "ConfigPeripheralsInterfaceKicker.hpp"
#include "configWorldModelSync.hpp"
#include "multiCamStatistics.hpp"
#include "SimulationScene.hpp"
#include "visionFrame.hpp"
#include "diagTrueBall.hpp"
#include "HeightmapNames.hpp"

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
typedef robotPosVel                               T_ROBOT_POSVEL_SETPOINT;
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
typedef int                                       T_HEARTBEAT;
typedef int                                       T_TP_HEARTBEAT;
typedef int                                       T_HEARTBEAT_COACH;
typedef int                                       T_HEARTBEAT_WORLDMODEL;
typedef int                                       T_SIMULATION_HEARTBEAT_DONE;
typedef robotStatusEnum                           T_INPLAY_FEEDBACK;
typedef std::vector<event>                        T_EVENT_LIST;
typedef diagWorldModelShared                      T_DIAG_WORLDMODEL_SHARED;
typedef diagWorldModelLocal                       T_DIAG_WORLDMODEL_LOCAL;
typedef diagTeamplay                              T_DIAG_TEAMPLAY;
typedef diagPathPlanning                          T_DIAG_PATHPLANNING;
typedef diagVelocityControl                       T_DIAG_VELOCITYCONTROL;
typedef DiagBallHandling                          T_DIAG_BALLHANDLING;
typedef diagPeripheralsInterface                  T_DIAG_PERIPHERALSINTERFACE;
typedef diagHealthSlow                            T_DIAG_HEALTH_SLOW;
typedef diagHealthFast                            T_DIAG_HEALTH_FAST;
typedef diagTrueBall                              T_DIAG_TRUE_BALL;
typedef std::string                               T_ROBOT_ROLE;
typedef std::map<int, std::string>                T_ROBOT_ROLES;
typedef intention                                 T_INTENTION;
typedef CompositeHeightmapName                    T_HEIGHTMAP;
typedef tpOverrideState                           T_TP_OVERRIDE_STATE;
typedef tpOverrideResult                          T_TP_OVERRIDE_RESULT;
typedef ConfigWorldModel                          T_CONFIG_WORLDMODEL;
typedef ConfigHeartBeatCoach                      T_CONFIG_HEARTBEATCOACH;
typedef ConfigExecution                           T_CONFIG_EXECUTION;
typedef ConfigTeamplay                            T_CONFIG_TEAMPLAY;
typedef ConfigPathPlanning                        T_CONFIG_PATHPLANNING;
typedef ConfigVelocityControl                     T_CONFIG_VELOCITYCONTROL;
typedef ConfigVelocityTransform                   T_CONFIG_VELOCITYTRANSFORM;
typedef ConfigBallHandling                        T_CONFIG_BALLHANDLING;
typedef ConfigMotionPlanning                      T_CONFIG_MOTIONPLANNING;
typedef ConfigShootPlanning                       T_CONFIG_SHOOTPLANNING;
typedef ConfigPeripheralsInterfaceMotors          T_CONFIG_PERIPHERALSINTERFACE_MOTORS;
typedef ConfigPeripheralsInterfaceBallHandlers    T_CONFIG_PERIPHERALSINTERFACE_BALLHANDLERS;
typedef ConfigPeripheralsInterfaceKicker          T_CONFIG_PERIPHERALSINTERFACE_KICKER;
typedef configWorldModelSync                      T_CONFIG_WORLDMODELSYNC;
typedef multiCamStatistics                        T_MULTI_CAM_STATISTICS;
typedef SimulationScene                           T_SIMULATION_SCENE;
typedef double                                    T_SIMULATION_TIME;
typedef int                                       T_SIMULATION_TICK;
typedef visionFrame                               T_VISION_FRAME;
typedef diagTrueBall                              T_DIAG_TRUE_BALL;

// !!!!!!!!
// NOTE: If you update this list, please also update the Wiki page: "Software Architecture" and keep things consistent
// !!!!!!!!

#endif


