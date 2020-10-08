 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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

#include "FalconsRtDB2.hpp"

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

    void calcBallPossession(ballPossessionTypeEnum &bpType, int &bpRobot);
    void ballPossessionOverrule(int bpRobot, rtime timeNow);
    Vector2D getBallHandlerPosition(int robotId);
    
    void updateDiagnostics();
};

#endif /* ADAPTERSCOLLECTOR_HPP_ */
