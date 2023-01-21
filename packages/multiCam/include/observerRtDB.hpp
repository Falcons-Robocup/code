// Copyright 2018-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * observerRtDB.hpp
 *
 *  Created on: Oct 10, 2018
 *      Author: Erik Kouters
 */

#ifndef OBSERVERRTDB_HPP_
#define OBSERVERRTDB_HPP_

#include <stdio.h>
#include "observer.hpp"

#include <boost/thread/thread.hpp>
#include "boost/thread/mutex.hpp"
#include "rtdbStructs.hpp"
#include "RtDB2.h"
#include "cDiagnostics.hpp"
#include "FalconsRTDB.hpp"

class observerRtDB: public observer
{

    public:
        observerRtDB(const uint robotID, const bool cameraCorrectlyMounted, const float minimumLockTime);
        virtual ~observerRtDB();

        virtual void update_own_position(std::vector<robotLocationType> robotLocations, double timestampOffset);
        virtual void update_own_ball_position(std::vector<ballPositionType> ballLocations, double timestampOffset);
        virtual void update_own_obstacle_position(std::vector<obstaclePositionType> obstacleLocations, double timestampOffset);
        virtual void update_own_ball_possession(const bool hasPossession);
        virtual void update_multi_cam_statistics(multiCamStatistics const &multiCamStats);

        RtDB2 *_rtdb;
        int _myRobotId;
        boost::mutex mtx;

        // data may be written asynchronously using the update* function
        // there is one thread which writes into RTDB hence triggering worldModel and the rest of the software
        T_VIS_BALL_POSSESSION     _visionBallPossession = false;
        T_LOCALIZATION_CANDIDATES _robotLocCandidates;
        T_OBSTACLE_CANDIDATES     _obstacleCandidates;
        T_BALL_CANDIDATES         _ballCandidates;
        T_MULTI_CAM_STATISTICS    _multiCamStats;

        boost::thread             _heartBeatThread; // here will be the effective heartBeat

    private:
        bool isSamePosition(const float x, const float y, const float theta);
        void initializeRtDB();
        bool heartBeatTick();
        void heartBeatLoop();

        int localizationUniqueObjectIDIdx;
        int ballUniqueObjectIDIdx;
        int obstacleUniqueObjectIDIdx;
        const int MAX_UNIQUEOBJID = 10000;
};

#endif
