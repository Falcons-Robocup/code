 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cMotionPlanningClient.cpp
 *
 *  Created on: Dec 01, 2018
 *      Author: Erik Kouters
 */

#include "tracing.hpp"
#include "cDiagnostics.hpp" // TRACE_ERROR

#include "int/cMotionPlanner.hpp"

#include "ext/cMotionPlanningClient.hpp"

#include "../../include/int/MP_WorldModelInterface.hpp"
#include "PathPlanningClient.hpp"
#include "int/cAllActions.hpp"

cMotionPlanner* _motionPlanner;
MP_WorldModelInterface* _wmInterface;
PathPlanningClient* _ppClient;
MP_RTDBOutputAdapter* _rtdbOutput;


cMotionPlanningClient::cMotionPlanningClient()
{

    try
    {
        _wmInterface = new MP_WorldModelInterface();
        _rtdbOutput = new MP_RTDBOutputAdapter();
        _ppClient = new PathPlanningClient();
        _motionPlanner = new cMotionPlanner(_wmInterface, _ppClient, _rtdbOutput);

    }
    catch (std::exception &e)
    {
        std::cerr << "Error occurred:" << e.what() << std::endl;
        TRACE_ERROR("Error occurred: %s", e.what());
    }
}

cMotionPlanningClient::~cMotionPlanningClient()
{
    delete _wmInterface;
    delete _ppClient;
    delete _motionPlanner;
}

T_ACTION_RESULT cMotionPlanningClient::executeAction(const T_ACTION actionData)
{
    TRACE_FUNCTION("");

    tprintf("executeAction %s position=[%6.2f, %6.2f, %6.2f] slow=%d bh=%d",
        enum2str(actionData.action), actionData.position.x, actionData.position.y, actionData.position.z, actionData.slow, actionData.ballHandlersEnabled);

    std::vector<std::string> params;
    switch(actionData.action)
    {
        case actionTypeEnum::MOVE:
        {
            _motionPlanner->setAction(MP_ActionMoveToTarget());
            params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
            params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
            params.push_back(boost::lexical_cast<std::string>(actionData.position.z)); // Rz actually
            params.push_back(boost::lexical_cast<std::string>((int)actionData.slow));
            params.push_back(boost::lexical_cast<std::string>((int)actionData.ballHandlersEnabled));
            break;
        }
        case actionTypeEnum::KICK:
        {
            _motionPlanner->setAction(MP_ActionKick());
            params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
            params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
            break;
        }
        case actionTypeEnum::PASS:
        {
            _motionPlanner->setAction(MP_ActionPassToTarget());
            params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
            params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
            break;
        }
        case actionTypeEnum::SHOOT:
        {
            _motionPlanner->setAction(MP_ActionShootAtTarget());
            params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
            params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
            params.push_back(boost::lexical_cast<std::string>(actionData.position.z));
            params.push_back("SHOOT");
            break;
        }
        case actionTypeEnum::LOB:
        {
            _motionPlanner->setAction(MP_ActionShootAtTarget());
            params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
            params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
            params.push_back(boost::lexical_cast<std::string>(actionData.position.z));
            params.push_back("LOB");
            break;
        }
        case actionTypeEnum::STOP:
        {
            _motionPlanner->setAction(MP_ActionStop());
            params.push_back(boost::lexical_cast<std::string>((int)actionData.ballHandlersEnabled));
            break;
        }
        case actionTypeEnum::GET_BALL:
        {
            _motionPlanner->setAction(MP_ActionGetBall());
            params.push_back(boost::lexical_cast<std::string>((int)actionData.slow));
            break;
        }
        case actionTypeEnum::TURN_AWAY_FROM_OPPONENT:
        {
            _motionPlanner->setAction(MP_ActionTurnAwayFromOpponent());
            params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
            params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
            params.push_back(boost::lexical_cast<std::string>((int)actionData.slow));
            break;
        }
        case actionTypeEnum::KEEPER_MOVE:
        {
            _motionPlanner->setAction(MP_ActionKeeperMove());
            params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
            break;
        }
        case actionTypeEnum::INTERCEPT_BALL:
        {
            _motionPlanner->setAction(MP_ActionInterceptBall());
            params.push_back(boost::lexical_cast<std::string>((int)actionData.slow));
            break;
        }
        default:
        {
            throw std::runtime_error("Unknown enum value received from RTDB ActionData");
            break;
        }
    }
    _motionPlanner->setActionParameters(params);

    T_ACTION_RESULT actionResult = _motionPlanner->execute();
    tprintf("             executeAction RESULT result=%s", enum2str(actionResult.result));
    return actionResult;
}
    
double cMotionPlanningClient::getTimeToBall(const uint8_t robotID)
{
    // First update WM data, then retrieve ball and robotState
    _wmClient.update();
    
    double distance = 999;

    if (_wmClient.getBalls().size() > 0)
    {
        ballResult ballRes = _wmClient.getBalls().at(0);
        Vector3D ball = Vector3D(ballRes.position.x, ballRes.position.y, ballRes.position.z);

        robotState robot;
        if (_wmClient.getRobotState(robot, robotID))
        {
            distance = calc_distance(ball.x, ball.y, robot.position.x, robot.position.y);
        }
        // TODO: what if robot is offline? do we trust teamplay won't call us then?

        // TODO: make a more accurate model here, by factoring in robot speed, acceleration, possibly even pathing
    }

    return distance;
}
