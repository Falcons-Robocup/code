// Copyright 2019-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionGetBall.cpp
 *
 *  Created on: Feb 3, 2018
 *      Author: Jan Feitsma
 */

#include "../include/int/MP_ActionGetBall.hpp"

using namespace std;

/* Get the ball.
 * This motionPlanning action combines legacy teamplay actions getBall and getBallOnVector.
 * 
 * Exit criteria:
 *  PASSED: robot has the ball
 *  FAILED: other teammember got the ball, or there is no ball, or robot is inactive
 *
 * Options:
 *   bool slow : move slowly towards the ball (typically used during setpiece)
 *
 * Method:
 *  * when chasing a closeby ball, project a target using the ball speed vector (legacy getBallOnVector)
 *  * TODO when moving towards a far-away ball, do a sprint, in case no obstacles closeby and if needed
 *
 * Interfacing with teamplay:
 *  * teamplay should assign 1 robot to get the ball (based on ball proximity, vector and/or intention)
 *    and that robot should be the only one calling this getBall action
 *    (if more robots are assigned, they soon end up fighting over the ball, which looks silly and is inefficient)
 *  * teamplay is responsible for outside-field boundaries, motionPlanning will just try to get the ball, regardless...
 *
 * Future: 
 *  * adjust target to take ball bouncing into account
 * 
 */

void MP_ActionGetBall::unpackParameters()
{
    _motionType = (motionTypeEnum)(boost::lexical_cast<int>(_params.at(0)));
}

void MP_ActionGetBall::getCfg()
{
    _obstacleThreshold  = getConfig().getBallConfig.obstacleThreshold;
    _ballSpeedThreshold = getConfig().getBallConfig.ballSpeedThreshold;
    _ballSpeedScaling   = getConfig().getBallConfig.ballSpeedScaling;
}

void MP_ActionGetBall::initialize()
{

    getCfg();
    _rtdbOutput->setBallHandlersSetpoint(true);
    
    _R  = _wm->getPosition();
    _Vr = _wm->getVelocity();
    _B  = _wm->ballPosition();
    _Vb = _wm->ballVelocity();
    // face ball
    _target = _R;
    faceBall();
}

void MP_ActionGetBall::analyzeGeometry()
{
    // analyze ball movement
    Velocity2D ballVelocity(_wm->ballVelocity().x, _wm->ballVelocity().y, 0);
    _ballMovingFastEnough = (vectorsize(Vector2D(ballVelocity.x, ballVelocity.y)) > _ballSpeedThreshold);

    // roadmap: detect if ball is bouncing, if so, adjust depth D on the ball line

    ////// !! 2020-12-31 EKPC -- WIP for overtaking ball by sprinting
    // see if we can overtake the ball and catch it with a sprint
    //for (int i = 0; i < 10; i++)
    //{
    //    Vector2D ballPosAfterXSeconds = Vector2D(_B.x, _B.y) + Vector2D(_wm->ballVelocity().x, _wm->ballVelocity().y) * double(i);
    //    Vector2D robotPosAfterXSeconds = Vector2D(_R.x, _R.y) + Vector2D(_wm->ballVelocity().x, _wm->ballVelocity().y).normalized() * 1.6 * double(i);
    //    // overtake at +1m
    //    Vector2D overtakeBallPos = Vector2D(ballPosAfterXSeconds.x, ballPosAfterXSeconds.y) + Vector2D(_wm->ballVelocity().x, _wm->ballVelocity().y).normalized() * 1.0;

    //    std::stringstream str;
    //    str << "------\n";
    //    str << "After " << i << " seconds:\n";
    //    str << "ballPos=(" << ballPosAfterXSeconds.x << ", " << ballPosAfterXSeconds.y << ")\n";
    //    str << "robotPos=(" << robotPosAfterXSeconds.x << ", " << robotPosAfterXSeconds.y << ")\n";
    //    str << "overtakeBallPos=(" << overtakeBallPos.x << ", " << overtakeBallPos.y << ")\n";
    //    str << "------\n";
    //    
    //    TRACE(str.str().c_str());
    //}
}

void MP_ActionGetBall::faceBall()
{
    _target.phi = angle_between_two_points_0_2pi(_R.x, _R.y, _B.x, _B.y);
}

actionResultTypeEnum MP_ActionGetBall::execute()
{
    TRACE_FUNCTION("");

    // check active
    if (!_wm->isActive())
    {
        TRACE("Failed: WM inactive");
        return actionResultTypeEnum::FAILED;
    }
    // check ball existence
    if (_wm->noBall())
    {
        TRACE("Failed: Ball not found");
        return actionResultTypeEnum::FAILED;
    }
    // check ball possession
    if (_wm->hasBall())
    {
        TRACE("Robot has ball");
        // action has finished successfully
        // notify pathPlanning to reset, to clear diagnostics and to prevent setpoint drift (watchdog)
        // (this might actually have been contributing to the rotation-on-intercept issue we have been seeing)
        // TODO: check if this works OK for self-pass
        return setMotionSetpointAndCalculate(actionTypeEnum::STOP, Position2D(), motionTypeEnum::NORMAL);
    }
    if (_wm->teamHasBall())
    {
        TRACE("Failed: Team already has ball");
        return actionResultTypeEnum::FAILED;
    }

    unpackParameters();

    initialize();
    if (_wm->opponentHasBall())
    {
        // TODO - compute if we need to place a subtarget in front of the ball before grabbing it.
        _target.x = _B.x;
        _target.y = _B.y;
        TRACE("harass (%.2f, %.2f)", _target.x, _target.y);
    }
    else
    {
        // ball is free on the field
    
        // analyze the situation
        analyzeGeometry();
        
        // choose what kind of move to do: getBall or getBallOnVector
        if (_ballMovingFastEnough)
        {
            // getBallOnVector
            Vector2D tmp = Vector2D(_B.x, _B.y) + Vector2D(_wm->ballVelocity().x, _wm->ballVelocity().y) * _ballSpeedScaling;
            _target.x = tmp.x;
            _target.y = tmp.y;
            TRACE("getBallOnVector (%.2f, %.2f)", _target.x, _target.y);
        }
        else
        {
            // getBall
            _target.x = _B.x;
            _target.y = _B.y;
            TRACE("getBall (%.2f, %.2f)", _target.x, _target.y);
        }
    }
    // execute the move
    faceBall();
    TRACE("target (%.2f, %.2f, %.2f)", _target.x, _target.y, _target.phi);
    return setMotionSetpointAndCalculate(actionTypeEnum::MOVE, _target, _motionType);
}

