 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * RTDBaccess.cpp
 *
 *  Created on: Feb 13, 2019
 *      Author: Coen Tempelaars
 */

#include "int/RTDBaccess.hpp"

#include <map>
#include "falconsCommon.hpp"

static const std::map<RobotID, int> ROBOT_NUMBERS =
{
    {RobotID::r1, 1},
    {RobotID::r2, 2},
    {RobotID::r3, 3},
    {RobotID::r4, 4},
    {RobotID::r5, 5}
};

static const std::map<TeamID, std::string> PATHS =
{
        {TeamID::A, RTDB2_SIM_TEAM_A_PATH},
        {TeamID::B, RTDB2_SIM_TEAM_B_PATH}
};

RtDB2* getRTDBConnection()
{
    return RtDB2Store::getInstance().getRtDB2(COACH_AGENTID);
}

RtDB2* getRTDBConnection (const TeamID& teamID)
{
    auto path = PATHS.at(teamID);
    return RtDB2Store::getInstance().getRtDB2(COACH_AGENTID, path);
}

RtDB2* getRTDBConnection (const TeamID& teamID, const RobotID& robotID)
{
    auto robotNumber = ROBOT_NUMBERS.at(robotID);
    auto path = PATHS.at(teamID);
    return RtDB2Store::getInstance().getRtDB2(robotNumber, path);
}

int getRobotNumber (const RobotID& robotID)
{
    return ROBOT_NUMBERS.at(robotID);
}
