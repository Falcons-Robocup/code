 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRTDBInputAdapter.cpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#include "int/adapters/cRTDBInputAdapter.hpp"
//
#include "FalconsCommon.h" //getRobotNumber(), getTeamChar()
#include "cDiagnostics.hpp"
#include "tracing.hpp"

cRTDBInputAdapter::cRTDBInputAdapter(cMotionPlanner* mp)
{
    TRACE_FUNCTION("");
    _myRobotId = getRobotNumber();
    auto teamChar = getTeamChar();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId, teamChar);

    _motionPlanner = mp;
}

cRTDBInputAdapter::~cRTDBInputAdapter()
{
}

void cRTDBInputAdapter::waitForActionData()
{
    while (true)
    {
        int result = _rtdb->waitForPut(K_ACTION);
        if (result == RTDB2_SUCCESS)
        {
            getActionData();
        }

        WRITE_TRACE;
    }
}

void cRTDBInputAdapter::getActionData()
{
    TRACE_FUNCTION("");
    T_ACTION actionData;

    int r = _rtdb->get(K_ACTION, &actionData);

    tprintf("get ACTION %s position=[%6.2f, %6.2f, %6.2f] slow=%d bh=%d id=%d", 
        enum2str(actionData.action), actionData.position.x, actionData.position.y, actionData.position.z, actionData.slow, actionData.ballHandlersEnabled, actionData.id);

    if (r == RTDB2_SUCCESS)
    {
        std::vector<std::string> params;
        switch(actionData.action)
        {
            case actionTypeEnum::MOVE:
            {
                _motionPlanner->setAction(cActionMoveToTarget());
                params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
                params.push_back(boost::lexical_cast<std::string>((int)actionData.slow));
                params.push_back(boost::lexical_cast<std::string>((int)actionData.ballHandlersEnabled));
                break;
            }
            case actionTypeEnum::KICK:
            {
                _motionPlanner->setAction(cActionKick());
                params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
                break;
            }
            case actionTypeEnum::PASS:
            {
                _motionPlanner->setAction(cActionPassToTarget());
                params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
                break;
            }
            case actionTypeEnum::SHOOT:
            {
                _motionPlanner->setAction(cActionShootAtTarget());
                params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.z));
                params.push_back("SHOOT");
                break;
            }
            case actionTypeEnum::LOB:
            {
                _motionPlanner->setAction(cActionShootAtTarget());
                params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.z));
                params.push_back("LOB");
                break;
            }
            case actionTypeEnum::STOP:
            {
                _motionPlanner->setAction(cActionStop());
                break;
            }
            case actionTypeEnum::GET_BALL:
            {
                _motionPlanner->setAction(cActionGetBall());
                params.push_back(boost::lexical_cast<std::string>((int)actionData.slow));
                break;
            }
            case actionTypeEnum::TURN_AWAY_FROM_OPPONENT:
            {
                _motionPlanner->setAction(cActionTurnAwayFromOpponent());
                params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
                params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
                params.push_back(boost::lexical_cast<std::string>((int)actionData.slow));
                break;
            }
            case actionTypeEnum::KEEPER_MOVE:
            {
                _motionPlanner->setAction(cActionKeeperMove());
                params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
                break;
            }
            default:
            {
                throw std::runtime_error("Unknown enum value received from RTDB ActionData");
                break;
            }
        }
        _motionPlanner->setActionParameters(params);
        _motionPlanner->setActionId(actionData.id);
        _motionPlanner->execute();
    }
}
