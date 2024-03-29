// Copyright 2019-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionInterceptBall.cpp
 *
 *  Created on: Apr 23, 2018
 *      Author: Erik Kouters
 */

#include "../include/int/MP_ActionInterceptBall.hpp"

//#include "int/algorithms/intercept.hpp"
#include "intersect.hpp"
#include "velocity2d.hpp"

using namespace std;

/* Intercept the ball.
 * Definition: an intercept is a sideways move which faces the ball, so the robot is not facing forward along with its driving direction.
 *
 * Exit criteria:
 *  PASSED: robot has the ball
 *  FAILED: other teammember got the ball, or there is no ball, or robot is inactive
 *
 * Options:
 *   bool slow : move slowly towards the ball (typically used during setpiece)
 *
 * Method:
 *  * intercept if the ball is sufficiently far away and moving towards the robot
 *  * when chasing a closeby ball, project a target using the ball speed vector (legacy getBallOnVector)
 *  * TODO when moving towards a far-away ball, do a sprint, in case no obstacles closeby and if needed
 *
 * Interfacing with teamplay:
 *  * teamplay cannot put multiple robots in intercept mode anymore
 *  * teamplay should assign 1 robot to get the ball (based on ball proximity, vector and/or intention)
 *    and that robot should be the only one calling this getBall action
 *    (if more robots are assigned, they soon end up fighting over the ball, which looks silly and is inefficient)
 *  * teamplay is responsible for outside-field boundaries, motionPlanning will just try to get the ball, regardless...
 *
 * Future:
 *  * if opponent is closeby and likely to intercept, then step in front of the opponent (dutch: 'voor de man komen')
 *    although one could argue that dynamic positioning & pass choice using height-maps should prevent this scenario?
 *  * adjust target to take ball bouncing into account
 *
 */

//////////////////////
// PUBLIC FUNCTIONS //
//////////////////////

void MP_ActionInterceptBall::unpackParameters()
{
}

actionResultTypeEnum MP_ActionInterceptBall::execute()
{
    TRACE_FUNCTION("");

    // check active
    if (!_wm->isActive())
    {
        return actionResultTypeEnum::FAILED;
    }
    // check ball existence
    if (_wm->noBall())
    {
        return actionResultTypeEnum::FAILED;
    }
    // check ball possession
    if (_wm->hasBall())
    {   
        return actionResultTypeEnum::PASSED;
    }
    if (_wm->teamHasBall())
    {
        return actionResultTypeEnum::FAILED;
    }
    initialize();

    unpackParameters();

    // ball is free on the field, or might be close to opponent

    // analyze the situation
    analyzeGeometry();

    bool should_intercept = false;
    boost::optional<Position2D> intercept_position = boost::make_optional(false, Position2D());

    // If 'active intercept' is enabled (see motionPlanning.yaml)
    if (_activeIntercept)
    {
        intercept_position = activePositionIntercept( );

        TRACE("The ball is moving fast enough: ") << std::to_string(_ballMovingFastEnough)
            << "  The ball intersects capture radius: " << std::to_string(intercept_position.is_initialized());

        should_intercept = _ballMovingFastEnough && intercept_position.is_initialized();
    }
    else
    {
        intercept_position = positionIntercept( );

        TRACE("The ball is moving towards us: ") << std::to_string(_ballMovingTowardsUs)
            << "  The ball is moving fast enough: " << std::to_string(_ballMovingFastEnough)
            << "  The ball intersects capture radius: " << std::to_string(intercept_position.is_initialized());

        should_intercept = _ballMovingTowardsUs && _ballMovingFastEnough && intercept_position.is_initialized();
    }

    if (should_intercept)
    {
        _target.x = intercept_position->x;
        _target.y = intercept_position->y;
        _target.phi = _R.phi;

        TRACE("intercept (%.2f, %.2f)", _target.x, _target.y);
    }
    else
    {
        _target.x = _R.x;
        _target.y = _R.y;

        TRACE("wait for intercept at robot location (%.2f, %.2f)", _target.x, _target.y);
    }

    // execute the move

    faceBall();

    _rtdbOutput->setBallHandlersSetpoint(true);

    TRACE("target (%.2f, %.2f, %.2f)", _target.x, _target.y, _target.phi);
    // move by writing setpoint to RTDB and calling pathPlanning iteration
    // ignore pathPlanning pass/fail, as robot should continue intercepting even when if it is already perfectly positioned
    (void)setMotionSetpointAndCalculate(actionTypeEnum::MOVE, _target, motionTypeEnum::INTERCEPT);
    return actionResultTypeEnum::RUNNING;
}


///////////////////////
// PRIVATE FUNCTIONS //
///////////////////////

void MP_ActionInterceptBall::getCfg()
{
    _obstacleThreshold  = getConfig().interceptBallConfig.obstacleThreshold;
    _ballSpeedThreshold = getConfig().interceptBallConfig.ballSpeedThreshold;
    _ballSpeedScaling   = getConfig().interceptBallConfig.ballSpeedScaling;
    _captureRadius      = getConfig().interceptBallConfig.captureRadius;
    _activeIntercept    = getConfig().interceptBallConfig.activeIntercept;
}

void MP_ActionInterceptBall::initialize()
{

    getCfg();

    _R  = _wm->getPosition();
    _Vr = _wm->getVelocity();
    _B  = _wm->ballPosition();
    _Vb = _wm->ballVelocity();
    _target = _R;
}

void MP_ActionInterceptBall::analyzeGeometry()
{
    // analyze ball movement
    _ballMovingTowardsOurFront = isBallMovingTowardsOurFront();
    _ballMovingTowardsUs = isBallMovingTowardsUs();
    _ballMovingFastEnough = isBallMovingFastEnough();

    // calculate intercept
    //bool success = false;
    //float timeNeeded = 0.0;
    //Velocity2D targetVel; // unused
    //calculateIntercept(_R, _Vr, _B, _Vb, success, timeNeeded, _target, targetVel);

    // roadmap: detect if ball is bouncing, if so, adjust depth D on the ball line
}

bool MP_ActionInterceptBall::shouldIntercept()
{
    return _ballMovingTowardsUs && _ballMovingFastEnough;
}

void MP_ActionInterceptBall::faceBall()
{
    _target.phi = angle_between_two_points_0_2pi(_target.x, _target.y, _B.x, _B.y);
}

boost::optional<Position2D> MP_ActionInterceptBall::positionIntercept()
{
    // span a strafing line using RCS coordinates
    Position2D intercept_line_left_rcs(-_captureRadius, 0, 0);
    Position2D intercept_line_right_rcs(_captureRadius, 0, 0);

    // modify currentpos, as if already facing the ball
    Position2D robot_position_rotated_to_ball = _R;
    robot_position_rotated_to_ball.phi = angle_between_two_points_0_2pi(
        _R.x, _R.y,
        _B.x, _B.y);
    const Position2D intercept_line_left = intercept_line_left_rcs.transform_rcs2fcs(robot_position_rotated_to_ball);
    const Position2D intercept_line_right = intercept_line_right_rcs.transform_rcs2fcs(robot_position_rotated_to_ball);

    const Vector2D ball_position(_B.x, _B.y);
    Vector2D ball_projection = ball_position + (10.0 * Vector2D(_Vb.x, _Vb.y)); // make vector long enough

    // The position where the robot intersects with the ball trajectory.
    Vector2D intersectResult;
    if (intersect(ball_position, ball_projection, intercept_line_left.xy(), intercept_line_right.xy(), intersectResult))
    {
        if ((intersectResult - _R.xy()).size() < _captureRadius)
        {
            Position2D intercept_position;
            intercept_position.x = intersectResult.x;
            intercept_position.y = intersectResult.y;
            intercept_position.phi = robot_position_rotated_to_ball.phi;
            return intercept_position;
        }
    }
    return boost::none;
}

boost::optional<Position2D> MP_ActionInterceptBall::activePositionIntercept()
{
    const Vector2D own_location(_R.x, _R.y);
    // TODO use proper data from configuration
    const double kMaxRobotSpeed = 1.8;
    const Vector2D ball_location(_B.x, _B.y);
    const Vector2D ball_velocity(_Vb.x, _Vb.y);

    boost::optional<Vector2D> intercept_position = geometry::KinematicIntersect::intersect(
        own_location, kMaxRobotSpeed, ball_location, ball_velocity);

    if (!intercept_position.is_initialized())
    {
        return boost::none;
    }

    const Vector2D diff = *intercept_position - own_location;
    const double radius = diff.size();
    if (radius > _captureRadius)
    {
        TRACE("Outside capture radius = %lf", radius);
        return boost::none;
    }

    TRACE("Intercept destination = {%lf, %lf}", intercept_position->x, intercept_position->y);
    return Position2D(intercept_position->x, intercept_position->y, 0.0);
}

bool MP_ActionInterceptBall::isBallMovingTowardsOurFront()
{
    // TODO do something about duplicity in basic geometry types (Position2D versus Pose2D)
    geometry::Velocity2D ball_velocity_to_be_transformed(_Vb.x, _Vb.y, _Vb.z);
    const geometry::Pose2D robot_pose(_R.x, _R.y, _R.phi);
    ball_velocity_to_be_transformed.transformFCS2RCS(robot_pose);
    return ball_velocity_to_be_transformed.y < 0.0;
}

bool MP_ActionInterceptBall::isBallMovingTowardsUs()
{
    const Vector2D robot_position(_R.x, _R.y);
    const Vector2D ball_position(_B.x, _B.y);
    const Vector2D ball_velocity(_Vb.x, _Vb.y);
    const double cos_angle = (ball_position - robot_position) * ball_velocity;
    return cos_angle < 0.0; // -0.707
}

bool MP_ActionInterceptBall::isBallMovingFastEnough()
{
    const float ball_speed = vectorsize(_Vb);
    return ball_speed > _ballSpeedThreshold;
}
