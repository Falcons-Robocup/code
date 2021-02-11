// Copyright 2018-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBInputAdapter.hpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#ifndef RTDBINPUTADAPTER_HPP_
#define RTDBINPUTADAPTER_HPP_

#include <vector>

#include "FalconsRtDB2.hpp"
#include "cRtDBClient.hpp"

#include "int/facilities/identifierGenerator.hpp"

// WM's internal data administrators
#include "int/administrators/ballAdministrator.hpp"
#include "int/administrators/obstacleAdministrator.hpp"
#include "int/administrators/robotAdministrator.hpp"

// sharedTypes for vision data
#include "ballMeasurement.hpp"
#include "obstacleMeasurement.hpp"
#include "robotLocalizationMeasurement.hpp"
#include "ballPossessionTypeEnum.hpp"
#include "robotStatusEnum.hpp"

// sharedTypes for velocityControl data
#include "robotDisplacement.hpp"
#include "robotVelocity.hpp"

#define WMS_RTDB_TIMEOUT 7.0 // seconds - ignore if data is older

class RTDBInputAdapter
{
public:
    RTDBInputAdapter();
    ~RTDBInputAdapter();

    virtual void setBallAdministrator(ballAdministrator *ballAdmin);
    virtual void setObstacleAdministrator(obstacleAdministrator *obstacleAdmin);
    virtual void setRobotAdministrator(robotAdministrator *robotAdmin);

    // Data from Vision
    void getLocalizationCandidates();
    void getBallCandidates();
    void getObstacleCandidates();
    void getVisionBallPossession();

    // Data from VelocityControl
    void getRobotDisplacement(rtime timeNow);
    void getRobotVelocity(rtime timeNow);

    // Data from PeripheralsInterface
    void getInPlayState(rtime timeNow);

    // Data from BallHandling
    void getBallHandlingBallPossession();

    // Teammembers, needed for fake obstacle filtering
    void getTeamMembers();

    // Reconfiguration
    void updateConfig(T_CONFIG_WORLDMODELSYNC const &config);
    void enableInplayOverrule();

private:
    RtDB2 *_rtdb;
    int _myRobotId;
    cRtDBClient _rtdbClient;
    T_CONFIG_WORLDMODELSYNC _config;
    bool _inplayOverrule = false;

    // WM's internal data administrators
    ballAdministrator *_ballAdmin;
    obstacleAdministrator *_obstacleAdmin;
    robotAdministrator *_robotAdmin;

    identifierGenerator _uIDGenerator;

    T_BALL_CANDIDATES _ballCandidates;
    T_OBSTACLE_CANDIDATES _obstacleCandidates;
    T_LOCALIZATION_CANDIDATES _localizationCandidates;
    T_VIS_BALL_POSSESSION _visionBallPossession;
    robotStatusType _inPlay;
    T_ROBOT_DISPLACEMENT_FEEDBACK _prevRobotDisplacement;
    T_ROBOT_VELOCITY_FEEDBACK _robotVelocity;
    T_BALLHANDLERS_BALL_POSSESSION _ballIsCaughtByBallHandlers;

    bool _robotDisplacementInitialized;

    // helpers
    bool useVisionFromRobot(int robotId);
};

#endif
