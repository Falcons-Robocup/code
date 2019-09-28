 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
    void getRobotDisplacement();
    void getRobotVelocity();

    // Data from PeripheralsInterface
    void getInPlayState();

    // Data from BallHandling
    void getBallHandlingBallPossession();
    
    // Teammembers, needed for fake obstacle filtering
    void getTeamMembers();

    // Reconfiguration
    void updateConfig(T_CONFIG_WORLDMODELSYNC const &config);

private:
    RtDB2 *_rtdb;
    int _myRobotId;
    cRtDBClient _rtdbClient;
    T_CONFIG_WORLDMODELSYNC _config;

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
    T_ROBOT_DISPLACEMENT_FEEDBACK _robotDisplacements;
    T_ROBOT_VELOCITY_FEEDBACK _robotVelocity;
    T_BALLHANDLERS_BALL_POSSESSION _ballIsCaughtByBallHandlers;

    // helpers
    bool useVisionFromRobot(int robotId);
};

#endif

