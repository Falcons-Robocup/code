// Copyright 2016-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionInterceptBall.cpp
 *
 *  Created on: Jun 11, 2016
 *      Author: Jan Feitsma
 */

#include "int/actions/cActionInterceptBall.hpp"
#include "yaml-cpp/yaml.h"

#include "falconsCommon.hpp"
#include "pose2d.hpp"
#include "intersect.hpp"

#include "int/stores/ballStore.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/stores/robotStore.hpp"
#include "int/cWorldStateFunctions.hpp"
#include "cDiagnostics.hpp"

#include "boost/optional.hpp"

using namespace teamplay;

const std::string kCaptureRadiusKey = "captureRadius";

namespace hidden
{

}

cActionInterceptBall::cActionInterceptBall()
{
    _actionParameters[kCaptureRadiusKey] = std::make_pair(std::vector<std::string>{"float"}, true);
    _intention.action = actionTypeEnum::INTERCEPT_BALL;
}

// Execute ball intercept action. If the ball is coming towards the robot, then move to intercept. Otherwise, continuously face the ball.
// Returns: PASSED    if the robot managed to get the ball
//          RUNNING   if the robot does not yet have the ball
//          FAILED    if conditions are not met
// TODO: if ball was within radius, but then moved away, should we report failure?

// parameters:
// float captureRadius = radius in meters or 'tbd': get value from teamplayInterceptBall.yaml
behTreeReturnEnum cActionInterceptBall::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
        // workaround: if other robot is dribbling with the ball, intercepting robot should not respond
        if (doesTeamHaveBall(parameters))
        {
            return behTreeReturnEnum::FAILED;
        }

        if (teamplay::robotStore::getInstance().getOwnRobot().hasBall())
        {
            TRACE("PASSED");
            return behTreeReturnEnum::PASSED;
        }

        return interceptBall();

    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("cActionInterceptBall::execute Linked to: ") + e.what());
    }
    return behTreeReturnEnum::FAILED;
}

