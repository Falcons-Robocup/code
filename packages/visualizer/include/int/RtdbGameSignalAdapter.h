 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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

