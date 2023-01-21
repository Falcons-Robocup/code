// Copyright 2016-2022 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * adaptersCollector.hpp
 *
 *  Created on: Oct 6, 2016
 *      Author: Tim Kouters
 */

#ifndef ADAPTERSCOLLECTOR_HPP_
#define ADAPTERSCOLLECTOR_HPP_

#include <vector>
#include <map>

#include "FalconsRTDB.hpp"

#include "int/types/ball/ballType.hpp"
#include "diagWorldModel.hpp"
#include "int/types/obstacle/obstacleType.hpp"
#include "int/types/robot/robotDisplacementType.hpp"
#include "int/types/robot/robotStatusType.hpp"
#include "int/types/robot/robotType.hpp"
#include "int/types/robot/robotMeasurementType.hpp"

#include "int/administrators/ballAdministrator.hpp"
#include "int/administrators/obstacleAdministrator.hpp"
#include "int/administrators/robotAdministrator.hpp"

#include "int/adapters/RTDB/RTDBInputAdapter.hpp"
#include "int/adapters/RTDB/RTDBOutputAdapter.hpp"
#include "ext/RTDBConfigAdapter.hpp"

#define WMS_RTDB_TIMEOUT 7.0 // seconds - ignore if data is older

class adaptersCollector
{
public:
    adaptersCollector();
    ~adaptersCollector();

    void setBallAdministrator(ballAdministrator *ballAdmin);
    void setObstacleAdministrator(obstacleAdministrator *obstacleAdmin);
    void setRobotAdministrator(robotAdministrator *robotAdmin);
    void heartBeatRecalculation(rtime const timeNow);
    
    // RTDB functions
    void initializeRTDB();
    void setRTDBInputAdapter(RTDBInputAdapter *rtdbInputAdapter);
    void setRTDBOutputAdapter(RTDBOutputAdapter *rtdbOutputAdapter);
    void updateInputData();

    void reportToStdout();
    diagWorldModel getDiagnostics();

private:
    ballAdministrator *_ballAdmin;
    obstacleAdministrator *_obstacleAdmin;
    robotAdministrator *_robotAdmin;
    bool _ballIsCaughtByBallHandlers;
    robotStatusType _robotStatus;
    diagWorldModel _diagnostics;
    std::vector<ballClass_t> _balls; // for overruling ... TODO this entire SW component needs an overhaul

    RtDB2 *_rtdb;
    int _myRobotId;
    RTDBInputAdapter *_rtdbInputAdapter;
    RTDBOutputAdapter *_rtdbOutputAdapter;
    RTDBConfigAdapter _configAdapter;

    void calcSelfBallPossession(ballPossessionTypeEnum &bpType, int &bpRobot);
    void calcOpponentBallPossession(ballPossessionTypeEnum &bpType, int &bpRobot);
    void ballPossessionOverrule(int bpRobot, rtime timeNow);
    Vector2D getBallHandlerPosition(int robotId);
    
    void updateDiagnostics();
};

#endif /* ADAPTERSCOLLECTOR_HPP_ */
