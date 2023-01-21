// Copyright 2022 Edwin Schreuder (Falcons)
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
#include "FalconsRTDB.hpp"
#endif

// External:
#include "cDbConnection.hpp" // logger

// Internal:
#include "int/adapters/GameSignalAdapter.h"


class RtdbGameSignalAdapter : public GameSignalAdapter
{
    Q_OBJECT

    public:
        explicit RtdbGameSignalAdapter(std::shared_ptr<const cDbConnection> db_connection);
        ~RtdbGameSignalAdapter();

        void set_db_connection(std::shared_ptr<const cDbConnection> db_connection);

    private:
        std::shared_ptr<const cDbConnection> _db_connection;
        QTimer* _timer;
        rtime _startTimestamp;
        rtime _currentTimestamp;
        int _matchAge;
        void emitMatchState(int agentId);
        robotStatusEnum emitRobotState(int agentId);
        void emitRobotRoles(int agentId);
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

