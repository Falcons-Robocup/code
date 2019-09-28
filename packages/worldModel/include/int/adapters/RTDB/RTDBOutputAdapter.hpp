 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * RTDBOutputAdapter.hpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#ifndef RTDBOUTPUTADAPTER_HPP_
#define RTDBOUTPUTADAPTER_HPP_

#include <vector>

#include "FalconsRtDB2.hpp"

// WM's internal data administrators
#include "int/administrators/ballAdministrator.hpp"
#include "int/administrators/obstacleAdministrator.hpp"
#include "int/administrators/robotAdministrator.hpp"


class RTDBOutputAdapter
{
public:
    RTDBOutputAdapter();
    ~RTDBOutputAdapter();

    virtual void setBallAdministrator(ballAdministrator *ballAdmin);
    virtual void setObstacleAdministrator(obstacleAdministrator *obstacleAdmin);
    virtual void setRobotAdministrator(robotAdministrator *robotAdmin);

    void setRobotState();
    void setBalls(std::vector<ballClass_t> const &balls);
    void setObstacles();
    void setBallPossession(ballPossessionTypeEnum bpType, int bpRobot);

    // Reconfiguration
    void updateConfig(T_CONFIG_WORLDMODELSYNC const &config);

private:
    RtDB2 *_rtdb;
    int _myRobotId;
    ballPossessionTypeEnum _ballPossessionType;
    int _ballPossessionRobot = 0;
    T_CONFIG_WORLDMODELSYNC _config;

    // WM's internal data administrators
    ballAdministrator *_ballAdmin;
    obstacleAdministrator *_obstacleAdmin;
    robotAdministrator *_robotAdmin;

};

#endif

