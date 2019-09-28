 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
#include "cDiagnostics.hpp"
#include "FalconsRtDB2.hpp"

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
        T_VIS_BALL_POSSESSION     _visionBallPossession;
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
