// Copyright 2019-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionKeeperMove.cpp
 *
 *  Created on: Apr 25, 2018
 *      Author: Erik Kouters
 */

#include "../include/int/MP_ActionKeeperMove.hpp"

#include "falconsCommon.hpp"
#include <stdexcept>
#include "cDiagnostics.hpp"
#include "cEnvironmentField.hpp"


MP_ActionKeeperMove::MP_ActionKeeperMove()
{
    _result = actionResultTypeEnum::RUNNING;
    _disabledObstacleAvoidance = false;
    getCfg();
}

actionResultTypeEnum MP_ActionKeeperMove::execute()
// TODO: upon finish (destruction), re-enable obstacle avoidance, to not bump during park
{
    TRACE_FUNCTION("");

    // check basics conditions to return FAILED (robot not inplay)
    if (!_wm->isActive())
    {
        return actionResultTypeEnum::FAILED;
    }

    // unpack parameters
    unpackParameters();

    // clip the position such that the goalKeeper stays between the posts
    // also move slightly forward w.r.t. the line
    clipTarget();

    // get current position
    _currentPos = _wm->getPosition();

    // when inside goal area, the keeper must not respond anymore to obstacles
    // so once the keeper is close to goal, we disable obstacle avoidance by reconfiguring pathPlanning
    checkDisableObstacleAvoidance();

    // calculate
    calculateSlow();
    checkBallBehindKeeper();

    // determine if we need to extend keeperFrame
    checkExtendKeeperFrame();

    // move by writing setpoint to RTDB and calling pathPlanning iteration
    _result = setMotionSetpointAndCalculate(actionTypeEnum::MOVE, _targetPos, _motionType);

    // wrap up
    return _result;
}

void MP_ActionKeeperMove::clipTarget()
{
    poiInfo goalPostLeft = poiInfo();
    cEnvironmentField::getInstance().getFieldPOI(P_OWN_GOALPOST_LEFT, goalPostLeft);

    poiInfo goalPostRight = poiInfo();
    cEnvironmentField::getInstance().getFieldPOI(P_OWN_GOALPOST_RIGHT, goalPostRight);

    _targetPos.x = fmax(_targetPos.x, goalPostLeft.x + _GOALPOST_OFFSET_KEEPER);
    _targetPos.x = fmin(_targetPos.x, goalPostRight.x - _GOALPOST_OFFSET_KEEPER);

    // Use a fixed Y coordinate relative to the goal
    _targetPos.y = goalPostLeft.y + _Y_MAX_OFFSET_KEEPER;

    // Always face away from goal
    _targetPos.phi = M_PI_2;
}

void MP_ActionKeeperMove::unpackParameters()
{
    _targetPos.x = boost::lexical_cast<float>(_params.at(0));
}

void MP_ActionKeeperMove::getCfg()
{
    _Y_MAX_OFFSET_KEEPER = getConfig().keeperMoveConfig.yMaxOffset;
    _GOALPOST_OFFSET_KEEPER = getConfig().keeperMoveConfig.xGoalpostOffset;
    _haveKeeperFrame = (getRobotNumber() == 1);
    // TODO: do we need some facility which knows about these hardware capabilities?
}

void MP_ActionKeeperMove::calculateSlow()
{
    // in case of a substitution / park, overrule slow motion profile
    // we don't want our fragile keeper bumping into things

    areaInfo goalArea;
    cEnvironmentField::getInstance().getFieldArea(areaName::A_OWN_GOALAREA_EXTENDED, goalArea); // EXTENDED includes the goal itself
    if ( cEnvironmentField::getInstance().isPositionInArea(_currentPos.x, _currentPos.y, goalArea) )
    {
        _motionType = motionTypeEnum::NORMAL;
    }
    else
    {
        _motionType = motionTypeEnum::SLOW;
    }
}

void MP_ActionKeeperMove::checkBallBehindKeeper()
{
    // overrule target.x to current setpoint in case ball is behind keeper
    // to make it possible to get ball after a goal, while keeper remains active (e.g. robotCLI / blockly keeper)
    
    // if no ball seen, do nothing
    if (_wm->noBall())
    {
        return;
    }

    // get ball data
    Vector3D ballPos = _wm->ballPosition();

    // calculate
    poiInfo goalPostLeft = poiInfo();
    cEnvironmentField::getInstance().getFieldPOI(P_OWN_GOALPOST_LEFT, goalPostLeft);
    float goalLineY = goalPostLeft.y;
    if (ballPos.y < goalLineY)
    {
        _targetPos.x = _currentPos.x;
    }
}

void MP_ActionKeeperMove::checkExtendKeeperFrame()
{
    // if no ball seen, do nothing
    if (_wm->noBall())
    {
        return;
    }

    // if we do not have a keeperFrame, then do nothing
    if (!_haveKeeperFrame)
    {
        return;
    }

    // get ball data
    Vector3D ballPos = _wm->ballPosition();
    Vector3D ballVel = _wm->ballVelocity();

    // don't respond to balls on far half, for instance being passed around
    if (ballPos.y > 0)
    {
        return; // do nothing - ball is too far away
    }

    // check if it likely to be a pass
    if (fabs(ballVel.x) > fabs(ballVel.y))
    {
        return; // do nothing - ball seems to be moving along X axis
    }

    // check ball speed towards goal
    if (ballVel.y > -_FRAME_EXTENSION_SPEED_THRESHOLD)
    {
        return; // do nothing - ball is moving too slow or not towards goalie, assume we can deal with it via motion only
    }

    // determine if we need to extend the frame
    // left/right: based on current distance on where we are and where we need to be
    // if significantly different, keeper needs to perform a dash, so it might as well extend the frame
    // NOTE: we do not worry about rapid/erratic setpoints, since we expect these are filtered by peripheralsInterface and/or firmware
    if (_currentPos.x + _FRAME_SIDE_EXTENSION_TOLERANCE < _targetPos.x)
    {
        _rtdbOutput->setKeeperFrameSetpoint(keeperFrameSetpointEnum::RIGHT);
    }
    else if (_currentPos.x - _FRAME_SIDE_EXTENSION_TOLERANCE > _targetPos.x)
    {
        _rtdbOutput->setKeeperFrameSetpoint(keeperFrameSetpointEnum::LEFT);
    }
    // otherwise: is ball in the air?
    else if (ballPos.z > _FRAME_TOP_EXTENSION_TOLERANCE)
    {
        _rtdbOutput->setKeeperFrameSetpoint(keeperFrameSetpointEnum::UP);
    }
}

void MP_ActionKeeperMove::checkDisableObstacleAvoidance()
{
    float y_delta_goal = fabs(_currentPos.y - _targetPos.y);
    // one time only
    if ((y_delta_goal < 1.0) && !_disabledObstacleAvoidance)
    {
        _rtdbOutput->disableObstacleAvoidance();
        _disabledObstacleAvoidance = true;
        // TODO: re-enable obstacle avoidance when action finishes (e.g. moving to side for substitution of PARK)
    }
}

