// Copyright 2018-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
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

    tprintf("executeAction %s position=[%6.2f, %6.2f, %6.2f] motionType=%s bh=%d",
        enum2str(actionData.action), actionData.position.x, actionData.position.y, actionData.position.z, enum2str(actionData.motionType), actionData.ballHandlersEnabled);

    // Set the action.
    // If the same action is already active, the old action is kept alive.
    // Otherwise, a new action is created.
    _motionPlanner->setAction(actionData.action);

    // For each action, retrieve the parameters and store for use when executing the action
    std::vector<std::string> params;
    switch(actionData.action)
    {
        case actionTypeEnum::MOVE:
        {
            params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
            params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
            params.push_back(boost::lexical_cast<std::string>(actionData.position.z)); // Rz actually
            params.push_back(boost::lexical_cast<std::string>((int)actionData.motionType));
            params.push_back(boost::lexical_cast<std::string>((int)actionData.ballHandlersEnabled));
            break;
        }
        case actionTypeEnum::KICK:
        {
            params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
            params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
            break;
        }
        case actionTypeEnum::PASS:
        {
            params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
            params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
            break;
        }
        case actionTypeEnum::SHOOT:
        {
            params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
            params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
            params.push_back(boost::lexical_cast<std::string>(actionData.position.z));
            params.push_back("SHOOT");
            break;
        }
        case actionTypeEnum::LOB:
        {
            params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
            params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
            params.push_back(boost::lexical_cast<std::string>(actionData.position.z));
            params.push_back("LOB");
            break;
        }
        case actionTypeEnum::STOP:
        {
            params.push_back(boost::lexical_cast<std::string>((int)actionData.ballHandlersEnabled));
            break;
        }
        case actionTypeEnum::GET_BALL:
        {
            params.push_back(boost::lexical_cast<std::string>((int)actionData.motionType));
            break;
        }
        case actionTypeEnum::TURN_AWAY_FROM_OPPONENT:
        {
            params.push_back(boost::lexical_cast<std::string>(actionData.position.x));
            params.push_back(boost::lexical_cast<std::string>(actionData.position.y));
            break;
        }
        case actionTypeEnum::KEEPER_MOVE:
        {
            break;
        }
        case actionTypeEnum::INTERCEPT_BALL:
        {
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
