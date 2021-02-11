// Copyright 2016-2020 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionGoalKeeper.cpp
 *
 *  Created on: May 4, 2016
 *      Author: Tim Kouters
 */
#include "int/actions/cActionGoalKeeper.hpp"

#include "cDiagnostics.hpp"
#include "falconsCommon.hpp"
#include "pose2d.hpp"
#include "velocity2d.hpp"

#include "int/stores/ballStore.hpp"
#include "int/stores/obstacleStore.hpp"
#include "int/stores/robotStore.hpp"
#include "int/stores/fieldDimensionsStore.hpp"
#include "int/cWorldModelInterface.hpp"


using namespace teamplay;

cActionGoalKeeper::cActionGoalKeeper()
{
    boost::assign::insert( _actionParameters )
        ("interceptCaptureRadius", std::make_pair(std::vector<std::string>{"float"}, true) )
        ("interceptMinimumSpeed", std::make_pair(std::vector<std::string>{"float"}, true))
        ;

    _intention.action = actionTypeEnum::KEEPER_MOVE;
}

cActionGoalKeeper::~cActionGoalKeeper()
{

}

// Always position between goal and ball
// Always face forwards (away from goal)
// If no ball was found, goto middle of the goal
// Returns: RUNNING   always, action does not end
behTreeReturnEnum cActionGoalKeeper::execute(const std::map<std::string, std::string> &parameters)
{
    // TODO: move this ball-geometry and logic from teamplay to motionPlanning?
    
    try
    {
        // parse input parameters
        getSettings();
        parseParameters(parameters);

        // get required basic data
        ball ball = ballStore::getBall();
        Position2D myPos = robotStore::getInstance().getOwnRobot().getPosition();
        Vector2D ballPos(ball.getPosition().x, ball.getPosition().y);

        // initialize output: target position to move to
        // by default (no ball visible) goto center of goal
        Position2D targetPos = Position2D(_goalCenter.x, _goalCenter.y + _Y_MAX_OFFSET_KEEPER, M_PI_2); // Face forward

        // determine how to move
        // 1. no ball                            -> stand still in middle of the goal
        // 2. ball moving fast towards goal      -> dynamic intercept
        // 3. opponent preparing to take penalty -> guess where the shot is aimed at, stand in front of it
        // 4. otherwise                          -> position between ball and goal (actually, a point behind the goal)
        
        if (!ball.isLocationKnown())
        {
            // case 1 -> don't need to do anything, target is already initialized OK
        }
        else
        {
            // case 2, 3 or 4
            // we will end in each case with an intersect calculation between goal line and 
            // a ray which eminates from the ball position (towards secondaryPos)
            // the direction of the ray depends on the case

            // initialize for case 4
            Vector2D secondaryPos(0, _goalCenter.y - 2.0);
            
            // check if case 2 applies
            geometry::Pose2D currentPose(myPos.x, myPos.y, myPos.phi);
            geometry::Velocity2D ballVelocity(ball.getVelocity().x, ball.getVelocity().y, 0.0);
            geometry::Velocity2D ballVelocityRCS = ballVelocity.transformFCS2RCS(currentPose);
            float ballSpeed = ballVelocity.size();
            bool ballIsMovingTowardsUs = (ballVelocityRCS.y < 0.0);
            bool ballMovingFastEnough = (ballSpeed > _interceptMinimumSpeed);
            if (ballIsMovingTowardsUs && ballMovingFastEnough && ball.isAtOwnSide())
            {
                secondaryPos = ballPos + 10.0 * (Vector2D)ballVelocity; // make vector long enough for intersection
            }
            else
            {
                // check if case 3 applies
                Position2D opponent;
                if (respondToOpponentAim(opponent))
                {
            		secondaryPos = opponentAimTarget(opponent);
                }
            }
            
            // calculate intersection based on secondaryPos
            Vector2D intersectResult;
            if (intersect(ballPos, secondaryPos, _leftPosGoalLine, _rightPosGoalLine, intersectResult))
            {
                targetPos.x = intersectResult.x;
                targetPos.y = intersectResult.y;
            }
            
            // clip the position such that the goalKeeper stays between the posts
            // also move slightly forward w.r.t. the line
            clipTarget(targetPos);
            
            // smoothening to prevent too aggressive moves, which in turn cause other instabilities
            // (loc & balltracking "bokkesprongen")
            //discretize(targetPos);
            //timeFilter(targetPos);
            
        }
        
        // send target position as intention (what's the point)
        _intention.position.x = targetPos.x;
        _intention.position.y = targetPos.y;
        sendIntention();

        // perform the move
        keeperMove(targetPos.x);
        return behTreeReturnEnum::RUNNING;
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("cActionGoalKeeper::execute Linked to: ") + e.what());
    }

    return behTreeReturnEnum::FAILED;
}

void cActionGoalKeeper::parseParameters(const std::map<std::string, std::string> &parameters)
{
    // parameter "interceptCaptureRadius" is the radius to capture the ball
    std::string interceptCaptureRadiusStr("interceptCaptureRadius");
    auto paramValPair = parameters.find(interceptCaptureRadiusStr);
    if (paramValPair != parameters.end())
    {
        std::string interceptCaptureRadiusVal = paramValPair->second;
        if (interceptCaptureRadiusVal.compare(emptyValue) != 0)
        {
            _interceptCaptureRadius = std::stod(interceptCaptureRadiusVal);
        }
    }

    // parameter "interceptMinimumSpeed" is the minimum speed to capture the ball
    std::string interceptMinimumSpeedStr("interceptMinimumSpeed");
    paramValPair = parameters.find(interceptMinimumSpeedStr);
    if (paramValPair != parameters.end())
    {
        std::string interceptMinimumSpeedVal = paramValPair->second;
        if (interceptMinimumSpeedVal.compare(emptyValue) != 0)
        {
            _interceptMinimumSpeed = std::stod(interceptMinimumSpeedVal);
        }
    }
    
    // TODO make utility function? above code is highly duplicate...
}

void cActionGoalKeeper::getSettings()
{
    _goalCenter = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OWN_GOALLINE_CENTER);
    // Goalpost parameters
    _goalPostLeft = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OWN_GOALPOST_LEFT);
    _goalPostRight = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OWN_GOALPOST_RIGHT);
    // projection of line on which goalkeeper moves, extended with one meter
    _leftPosGoalLine = Vector2D(fieldDimensionsStore::getFieldDimensions().getWidth() / -2.0 - _INTERSECT_X_EXTENSION, _goalCenter.y + _Y_MAX_OFFSET_KEEPER);
    _rightPosGoalLine = Vector2D(fieldDimensionsStore::getFieldDimensions().getWidth() / 2.0 + _INTERSECT_X_EXTENSION, _goalCenter.y + _Y_MAX_OFFSET_KEEPER);
    _penaltyPos = Vector2D(0, -6); // TODO this needs to be part of fieldPOI, no? currently isn't
}

void cActionGoalKeeper::clipTarget(Position2D &targetPos)
{
    targetPos.x = fmax(targetPos.x, _goalPostLeft.x + _GOALPOST_OFFSET_KEEPER);
    targetPos.x = fmin(targetPos.x, _goalPostRight.x - _GOALPOST_OFFSET_KEEPER);
    targetPos.y = _goalPostLeft.y + _Y_MAX_OFFSET_KEEPER;
}

void cActionGoalKeeper::discretize(Position2D &targetPos)
{
    // round onto a static grid
    float ox = _goalPostLeft.x + _GOALPOST_OFFSET_KEEPER;
    float width = _goalPostRight.x - 2 * _GOALPOST_OFFSET_KEEPER - _goalPostLeft.x;
    targetPos.x = ox + round(_DISCRETIZE_NUM_SEGMENTS * (targetPos.x - ox) / width) * width / _DISCRETIZE_NUM_SEGMENTS;
}

void cActionGoalKeeper::timeFilter(Position2D &targetPos)
{
    // static = HACK! feature request: let the object live... so we can use data members
    static float lastX = 0;
    static double lastT = 0;
    double t = ftime::now();
    double elapsed = t - lastT;
    if (elapsed < _SMOOTH_TIMEFILTER)
    {
        // force to remain the same
        targetPos.x = lastX;
    }
    else
    {
        // allow to change, store the change event
        if (targetPos.x != lastX)
        {
            lastX = targetPos.x;
            lastT = t;
        }
    }
}

bool cActionGoalKeeper::respondToOpponentAim(Position2D &opponentPos)
{
    return false; // JFEI/APOX 20171005 disabled for now
    // obstacles are projected a bit too close due to the vision algorithm 
    // (see upside-down T overlay in vision visualizer)
    // this contributes to highly sensitive keeper behavior
    // obstacle coordinate correction should in principle be solved at vision level (tuning / pixel magic / ...)
    // furthermore, motion/latency also contributes to sensitive keeper movement ('bokkesprongen')
    // so we better disable this functionality for now

    ball ball = ballStore::getBall();
    Vector2D ballPos(ball.getPosition().x, ball.getPosition().y);
    float ballSpeed = Vector2D(ball.getVelocity().x, ball.getVelocity().y).size();
    // decide if we should respond to a robot who is about to take a penalty
    // criteria: ball should be lying still, close to penalty spot, with an obstacle behind it
    bool result = false;
    Vector2D obstaclePos;
    float closestDist = 999;
    if ((ballSpeed < _OPPONENT_AIM_BALL_SPEED_THRESHOLD) && ((ballPos - _penaltyPos).size() < _OPPONENT_AIM_PENALTY_PROXIMITY))
    {
        // find closest obstacle to penalty position
        auto opponents = obstacleStore::getInstance().getAllObstacles();
        for (auto it = opponents.begin(); it != opponents.end(); ++it)
        {
            Vector2D tmp(it->getPosition().x, it->getPosition().y);
            if ((tmp - _penaltyPos).size() < closestDist)
            {
                obstaclePos = tmp;
                closestDist = (tmp - _penaltyPos).size();
            }
        }
        TRACE("obstacle=(%6.2f,%6.2f)", obstaclePos.x, obstaclePos.y);
        if ((closestDist < _OPPONENT_AIM_PENALTY_PROXIMITY) && (obstaclePos.y > ballPos.y))
        {
            // also fill in opponent position, for later use when calculating target pos based on intersection 
            opponentPos.x = obstaclePos.x;
            opponentPos.y = obstaclePos.y;
            opponentPos.phi = angle_between_two_points_0_2pi(obstaclePos.x, obstaclePos.y, ballPos.x, ballPos.y);
            result = true;
        }
    }
    return result;
}

Vector2D cActionGoalKeeper::opponentAimTarget(Position2D const &opponentPos)
{
    float angle = opponentPos.phi;
    float r = _OPPONENT_AIM_RAY_DISTANCE;
    return Vector2D(cos(angle) * r, sin(angle) * r) + Vector2D(opponentPos.x, opponentPos.y);
}

