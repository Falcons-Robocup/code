 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * RTDBInputAdapter.hpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#ifndef RTDBINPUTADAPTER_PATHPLANNING_HPP_
#define RTDBINPUTADAPTER_PATHPLANNING_HPP_

#include "int/InputInterface.hpp"
#include "cWorldModelClient.hpp"
#include "FalconsRtDB2.hpp"


class RTDBInputAdapter : public InputInterface
{
public:
    RTDBInputAdapter();
    ~RTDBInputAdapter();

    void fetch();
    void waitForMotionSetpoint();

    motionSetpoint              getMotionSetpoint();
    std::vector<forbiddenArea>  getForbiddenAreas();
    robotState                  getRobotState();
    std::vector<robotState>     getTeamMembers();
    std::vector<ballResult>     getBalls();
    std::vector<obstacleResult> getObstacles();

private:
    cWorldModelClient* _wmClient;
    RtDB2 *_rtdb;
    int _myRobotId;

    motionSetpoint              _motionSetpoint;
    std::vector<forbiddenArea>  _forbiddenAreas;
    robotState                  _robot;
    std::vector<robotState>     _teamMembers;
    std::vector<ballResult>     _balls;
    std::vector<obstacleResult> _obstacles;

};

#endif
