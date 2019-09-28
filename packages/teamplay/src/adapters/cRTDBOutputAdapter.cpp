 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRTDBOutputAdapter.cpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#include "int/adapters/cRTDBOutputAdapter.hpp"

#include "FalconsCommon.h" //getRobotNumber(), getTeamChar()
#include "tracing.hpp"

cRTDBOutputAdapter::cRTDBOutputAdapter()
{
    TRACE_FUNCTION("");
    _myRobotId = getRobotNumber();
    auto teamChar = getTeamChar();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId, teamChar);
}

void cRTDBOutputAdapter::setActionData(const T_ACTION& actionData)
{
    TRACE_FUNCTION("");

    std::map<actionTypeEnum, std::string> ACTIONTYPEENUM2STR; // TODO generate this?
    ACTIONTYPEENUM2STR[actionTypeEnum::UNKNOWN] = "UNKNOWN";
    ACTIONTYPEENUM2STR[actionTypeEnum::MOVE] = "MOVE";
    ACTIONTYPEENUM2STR[actionTypeEnum::PASS] = "PASS";
    ACTIONTYPEENUM2STR[actionTypeEnum::SHOOT] = "SHOOT";
    ACTIONTYPEENUM2STR[actionTypeEnum::LOB] = "LOB";
    ACTIONTYPEENUM2STR[actionTypeEnum::STOP] = "STOP";
    ACTIONTYPEENUM2STR[actionTypeEnum::GET_BALL] = "GET_BALL";
    ACTIONTYPEENUM2STR[actionTypeEnum::TURN_AWAY_FROM_OPPONENT] = "TURN_AWAY_FROM_OPPONENT";
    ACTIONTYPEENUM2STR[actionTypeEnum::KEEPER_MOVE] = "KEEPER_MOVE";
    
    tprintf("put ACTION slow=%d bh=%d id=%d position=[%6.2f, %6.2f, %6.2f] type=%s", actionData.slow, actionData.ballHandlersEnabled, actionData.id, actionData.position.x, actionData.position.y, actionData.position.z, ACTIONTYPEENUM2STR[actionData.action].c_str());
    _rtdb->put(K_ACTION, &actionData);  
}

void cRTDBOutputAdapter::clearForbiddenAreas()
{
    TRACE_FUNCTION("");
    T_FORBIDDEN_AREAS forbiddenAreas; // empty array
    _rtdb->put(FORBIDDEN_AREAS, &forbiddenAreas);
}

void cRTDBOutputAdapter::setForbiddenAreas(const T_FORBIDDEN_AREAS& forbiddenAreas)
{
    TRACE_FUNCTION("");

    _rtdb->put(FORBIDDEN_AREAS, &forbiddenAreas);
}

void cRTDBOutputAdapter::setRole(const T_ROBOT_ROLE& role)
{
    TRACE_FUNCTION("");

    _rtdb->put(ROBOT_ROLE, &role);
}

void cRTDBOutputAdapter::setIntention(const T_INTENTION& intention)
{
    TRACE_FUNCTION("");

    _rtdb->put(INTENTION, &intention);
}
