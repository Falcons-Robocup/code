// Copyright 2018-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RtdbGameSignalAdapter.hpp
 *
 * Sample RTDB contents on some frequency and dispatch signals to visualizer.
 *
 *  Created on: Aug 13, 2018
 *      Author: Jan Feitsma
 */

#ifndef RTDBGAMESIGNALADAPTER_HPP_
#define RTDBGAMESIGNALADAPTER_HPP_


#ifndef Q_MOC_RUN
#include "FalconsRtDB2.hpp"
#endif

// External:
#include "cDbConnection.hpp" // logger

// Internal:
#include "int/GameSignalAdapter.h"


class RtdbGameSignalAdapter : public GameSignalAdapter, cDbConnection
{
    Q_OBJECT

    public:
        RtdbGameSignalAdapter();
        ~RtdbGameSignalAdapter();
    private:
        QTimer* _timer;
        rtime _startTimestamp;
        rtime _currentTimestamp;
        int _matchAge;
        void emitMatchState(int agentId);
        robotStatusEnum emitRobotState(int agentId);
        void emitBallResults(int agentId);
        void emitObstacleResults(int agentId);
        void emitGaussianObstacleResults(int agentId);
        void emitTrueBall(int agentId);
        void emitBallCandidates(int agentId);
        void emitObstacleCandidates(int agentId);
        void emitEvents(int agentId);
        void emitWorldModelData(int agentId);
        void emitHealthData(int agentId);
        void emitTeamPlayData(int agentId);
        void emitHalmwData(int agentId);
        void emitPathPlanningDiagnostics(int agentId);
        void emitOutofPlay(int agentId);

    private Q_SLOTS:
        void spinOnce();
};

#endif

