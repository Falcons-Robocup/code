// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * AbstractAction.cpp
 *
 *  Created on: Apr 30, 2016
 *      Author: Erik Kouters
 */

#include "int/actionnodes/AbstractAction.hpp"

#include <boost/lexical_cast.hpp>
#include <stdexcept>

#include "int/rules/RuleAvoidAreas.hpp"

#include "int/stores/BallStore.hpp"
#include "int/stores/ConfigurationStore.hpp"
#include "int/stores/FieldDimensionsStore.hpp"
#include "int/stores/GameStateStore.hpp"
#include "int/stores/RobotStore.hpp"

#include "int/adapters/RTDBInputAdapter.hpp"
#include "int/adapters/RTDBOutputAdapter.hpp"

#include "falconsCommon.hpp"
#include "tracing.hpp"

using namespace teamplay;

/* Static definitions */
static boost::shared_ptr<RuleAvoidAreas> rule_p;
static boost::shared_ptr<Timer> timer_p;


//////////////////////
// PUBLIC FUNCTIONS //
//////////////////////


AbstractAction::AbstractAction()
{
    timer_p.reset(new Timer());
    rule_p.reset(new RuleAvoidAreas(timer_p));
}

AbstractAction::~AbstractAction()
{
}


/////////////////////////
// PROTECTED FUNCTIONS //
/////////////////////////

void AbstractAction::moveTo( const Point2D& targetPos, const motionTypeEnum& motionType )
{
    std::ostringstream ss;
    ss << "target: " << targetPos;
    TRACE_FUNCTION(ss.str().c_str());

    std::map<std::string, std::string> params;
    bool ballHandlersEnabled = true;

    bool isPrepareSetPiece = teamplay::GameStateStore::getInstance().getGameState().isPrepareSetPiece();
    bool isParkingSetPiece = teamplay::GameStateStore::getInstance().getGameState().isParkingSetPiece();
    if (isPrepareSetPiece || isParkingSetPiece)
    {
        ballHandlersEnabled = false;
    }

    geometry::Pose2D myPos = RobotStore::getInstance().getOwnRobot().getPosition();
    geometry::Pose2D pose(targetPos.x, targetPos.y, myPos.Rz);

    // If we do not have the Ball: Always face Ball.
    auto Robot = RobotStore::getInstance().getOwnRobot();
    if (!Robot.hasBall())
    {
        auto Ball = BallStore::getBall();
        if (Ball.isLocationKnown())
        {
            auto ballPos = BallStore::getBall().getPosition();
            pose.Rz = angle_between_two_points_0_2pi(myPos.x, myPos.y, ballPos.x, ballPos.y);
        }
    }

    setForbiddenAreas();

    
    T_ACTION actionData = makeAction();
    actionData.action = actionTypeEnum::MOVE;
    actionData.position.x = pose.x;
    actionData.position.y = pose.y;
    actionData.position.z = pose.Rz;
    actionData.motionType = motionType;
    actionData.ballHandlersEnabled = ballHandlersEnabled;

    executeAction(actionData);
}

void AbstractAction::pass(const Point2D& target)
{
    std::ostringstream ss;
    ss << "target: " << target;
    TRACE_FUNCTION(ss.str().c_str());

    T_ACTION actionData = makeAction();
    actionData.action = actionTypeEnum::PASS;
    actionData.position.x = target.x;
    actionData.position.y = target.y;

    executeAction(actionData);
}

void AbstractAction::shoot(const Point2D& target)
{
    std::ostringstream ss;
    ss << "target: (" << target.x << ", " << target.y << ")";
    TRACE_FUNCTION(ss.str().c_str());

    T_ACTION actionData = makeAction();
    actionData.action = actionTypeEnum::SHOOT;
    actionData.position.x = target.x;
    actionData.position.y = target.y;
    actionData.position.z = 0.0;
    TRACE("Shooting at target: ") << std::to_string(target.x) << ", " << std::to_string(target.y);
    executeAction(actionData);
}

void AbstractAction::lobShot(const Point2D& target)
{
    std::ostringstream ss;
    ss << "target: (" << target.x << ", " << target.y << ")";
    TRACE_FUNCTION(ss.str().c_str());

    T_ACTION actionData = makeAction();
    actionData.action = actionTypeEnum::LOB;
    actionData.position.x = target.x;
    actionData.position.y = target.y;
    actionData.position.z = 0.0;

    TRACE("Lob Shooting at target: ") << std::to_string(target.x) << ", " << std::to_string(target.y);
    executeAction(actionData);
}

void AbstractAction::getBall(const motionTypeEnum& motionType)
{
    TRACE_FUNCTION("");

    setForbiddenAreas();

    T_ACTION actionData = makeAction();
    actionData.action = actionTypeEnum::GET_BALL;
    actionData.motionType = motionType;
    executeAction(actionData);
}

void AbstractAction::turnAwayFromOpponent(const Point2D& opponentPos)
{
    std::ostringstream ss;
    ss << "opponentPos: " << opponentPos;
    TRACE_FUNCTION(ss.str().c_str());

    geometry::Pose2D my_pos = RobotStore::getInstance().getOwnRobot().getPosition();
    double angle_to_opponent = angle_between_two_points_0_2pi(my_pos.x, my_pos.y, opponentPos.x, opponentPos.y);
    double opponent_angle_to_ball = project_angle_0_2pi(my_pos.Rz - angle_to_opponent);
    double opposite_angle_diff = project_angle_0_2pi(M_PI - opponent_angle_to_ball);

    // make angle from -pi to pi
    if (opposite_angle_diff > M_PI)
    {
        opposite_angle_diff -= 2*M_PI;
    }

    if (std::abs(opposite_angle_diff) < M_PI/20)
    {
        // If robot is already close to the opposite side of the oponent and we
        // still want to keep opposite to it, try to move around it instead of just turning in place

        double angle_delta = -M_PI/20;
        if (my_pos.Rz < M_PI/2 || my_pos.Rz > 3*M_PI/2)
        {
            angle_delta = -angle_delta;
        }

        double angle_from_opponent = angle_between_two_points_0_2pi(opponentPos.x, opponentPos.y, my_pos.x, my_pos.y);
        double target_angle_from_opponent = angle_from_opponent + angle_delta;

        double offset_size = 0.5;
        Vector2D target_offset(cos(target_angle_from_opponent), sin(target_angle_from_opponent));
        target_offset = target_offset * offset_size;

        Point2D target_pos = opponentPos + target_offset;
        double target_orientation = angle_between_two_points_0_2pi(opponentPos.x, opponentPos.y, target_pos.x, target_pos.y);

        T_ACTION actionData = makeAction();
        actionData.action = actionTypeEnum::MOVE;
        actionData.position.x = target_pos.x;
        actionData.position.y = target_pos.y;
        actionData.position.z = target_orientation;
        actionData.motionType = motionTypeEnum::WITH_BALL;
        actionData.ballHandlersEnabled = true;

        executeAction(actionData);
    }
    else
    {
        // turn in place

        T_ACTION actionData = makeAction();
        actionData.action = actionTypeEnum::TURN_AWAY_FROM_OPPONENT;
        actionData.position.x = opponentPos.x;
        actionData.position.y = opponentPos.y;
        executeAction(actionData);
    }
}

void AbstractAction::keeperMove()
{
    TRACE_FUNCTION("");

    T_ACTION actionData = makeAction();
    actionData.action = actionTypeEnum::KEEPER_MOVE;
    executeAction(actionData);
}

void AbstractAction::interceptBall()
{
    TRACE_FUNCTION("");

    T_ACTION actionData = makeAction();
    actionData.action = actionTypeEnum::INTERCEPT_BALL;
    executeAction(actionData);
}

void AbstractAction::stop()
{
    /* The "stop" must (1) move to current position and (2) disable motion and the HAL
    * Point (1) is important, otherwise motion will continue pursuing its current setpoint
    * until the setpoint watchdog interferes 
    * 
    * These details are now moved to motionPlanning.
    */

    T_ACTION actionData = makeAction();
    actionData.action = actionTypeEnum::STOP;
    executeAction(actionData);
}

bool AbstractAction::positionReached(double x, double y, double xy_threshold)
{
    geometry::Pose2D myPos = RobotStore::getInstance().getOwnRobot().getPosition();

    TRACE(" x: ") << std::to_string(x)
        << " y: " << std::to_string(y)
        << " xy threshold: " << std::to_string(xy_threshold);

    bool positionReached = ((fabs(myPos.x - x) < xy_threshold) && (fabs(myPos.y - y) < xy_threshold));

    TRACE("position reached: ") << ((positionReached) ? ("yes") : ("no"));

    return positionReached;
}

behTreeReturnEnum AbstractAction::getActionResult()
{
    return translateActionResultToTreeEnum( tpRTDBInputAdapter::getInstance().getActionResult() );
}


///////////////////////
// PRIVATE FUNCTIONS //
///////////////////////


T_ACTION AbstractAction::makeAction()
{
    T_ACTION actionData;
    actionData.action = actionTypeEnum::UNKNOWN;
    actionData.position.x = 0.0;
    actionData.position.y = 0.0;
    actionData.position.z = 0.0;
    actionData.motionType = motionTypeEnum::INVALID;
    actionData.ballHandlersEnabled = false;
    return actionData;
}

void AbstractAction::setForbiddenAreas()
{
    TRACE_FUNCTION("");
    auto forbidden_areas = getForbiddenAreas();
    // Convert std::vector<polygon2D> to std::vector<forbiddenArea>
    T_FORBIDDEN_AREAS forbiddenAreas;
    for (auto itArea = forbidden_areas.begin(); itArea != forbidden_areas.end(); ++itArea)
    {
        forbiddenArea newForbiddenArea;

        auto points = itArea->getPoints();
        for (auto it = points.begin(); it != points.end(); ++it)
        {
            vec2d point;
            point.x = it->x;
            point.y = it->y;
            newForbiddenArea.points.push_back(point);
        }

        forbiddenAreas.push_back(newForbiddenArea);
    }
    tpRTDBOutputAdapter::getInstance().setForbiddenAreas(forbiddenAreas);
}

std::vector<polygon2D> AbstractAction::getForbiddenAreas() const
{
    TRACE_FUNCTION("");
    auto forbiddenAreas = rule_p->getForbiddenAreas();
    TRACE("FBA: got %d forbiddenAreas from rules", (int)forbiddenAreas.size());

    /* We've had situations in the past where two robots were both sufficiently
    *  close enough to the ball to consider themselves as closest to the ball,
    *  resulting in two robots attempting to get the ball at the same time.
    */
    if (isAnyRobotGettingBall())
    {
        BallStore::getBall().avoid();
    }

    if (BallStore::getBall().mustBeAvoided())
    {
        auto ballPosition = BallStore::getBall().getPosition();
        polygon2D squareAroundBall;
        squareAroundBall.addPoint(ballPosition.x - 0.25, ballPosition.y - 0.25);
        squareAroundBall.addPoint(ballPosition.x - 0.25, ballPosition.y + 0.25);
        squareAroundBall.addPoint(ballPosition.x + 0.25, ballPosition.y + 0.25);
        squareAroundBall.addPoint(ballPosition.x + 0.25, ballPosition.y - 0.25);


        forbiddenAreas.push_back(squareAroundBall);
        TRACE("FBA: avoiding Ball as well");
    }

    /* Check whether other Robot is shooting on goal
    * If so, add forbidden area on those coordinates
    */
    std::vector<polygon2D> forbiddenActionAreas = getForbiddenActionAreas();
    TRACE("FBA: avoiding %d action areas", (int)forbiddenActionAreas.size());

    forbiddenAreas.insert(forbiddenAreas.begin(), forbiddenActionAreas.begin(), forbiddenActionAreas.end());

    TRACE("FBA: total %d forbiddenAreas", (int)forbiddenAreas.size());
    return forbiddenAreas;
}

std::vector<polygon2D> AbstractAction::getForbiddenActionAreas() const
{
    TRACE_FUNCTION("");
    std::vector<polygon2D> retVal;
    // Check shooting on goal
    auto teammembers = RobotStore::getInstance().getAllRobotsExclOwnRobot();
    for(auto it = teammembers.begin(); it != teammembers.end(); it++)
    {
        auto intention = tpRTDBInputAdapter::getInstance().getIntention(it->getNumber());
        if (intention.action == actionTypeEnum::SHOOT || intention.action == actionTypeEnum::LOB || intention.action == actionTypeEnum::PASS)
        {
            polygon2D shootArea;
            shootArea.addPoint(intention.position.x - 0.25, intention.position.y - 0.25);
            shootArea.addPoint(intention.position.x + 0.25, intention.position.y + 0.25);
            shootArea.addPoint(it->getPosition().x + 0.25, it->getPosition().y + 0.25);
            shootArea.addPoint(it->getPosition().x - 0.25, it->getPosition().y - 0.25);

            retVal.push_back(shootArea);
        }
    }

    return retVal;
}

behTreeReturnEnum AbstractAction::translateActionResultToTreeEnum(T_ACTION_RESULT actionResult) const
{
    behTreeReturnEnum result = behTreeReturnEnum::RUNNING;
    switch (actionResult.result)
    {
        case actionResultTypeEnum::INVALID:
        {
            result = behTreeReturnEnum::INVALID;
            break;
        }
        case actionResultTypeEnum::PASSED:
        {
            result = behTreeReturnEnum::PASSED;
            break;
        }
        case actionResultTypeEnum::FAILED:
        {
            result = behTreeReturnEnum::FAILED;
            break;
        }
        case actionResultTypeEnum::RUNNING:
        {
            result = behTreeReturnEnum::RUNNING;
            break;
        }
    }
    return result;
}


void AbstractAction::executeAction(T_ACTION& actionData) const
{
    // if our position is not according to the rules, move towards center
    // e.g., while standing in penalty area for too long
    if (actionData.action != actionTypeEnum::STOP)
    {
        moveToCenterIfCurrentPositionInvalid(actionData);
    }

    sendIntention(actionData);

    // T_ACTION is picked up by motionPlanning
    tpRTDBOutputAdapter::getInstance().setActionData(actionData);
}

void AbstractAction::moveToCenterIfCurrentPositionInvalid(T_ACTION& actionData) const
{
    // if our position is not according to the rules, move towards center
    // e.g., while standing in penalty area for too long
    if ( !rule_p->isCurrentPositionValid() )
    {
        actionData.action = actionTypeEnum::MOVE;
        actionData.position.x = 0.0;
        actionData.position.y = 0.0;
        actionData.motionType = motionTypeEnum::NORMAL;
    }
}

bool AbstractAction::isAnyRobotGettingBall() const
{
    TRACE_FUNCTION("");

    auto teammembers = RobotStore::getInstance().getAllRobotsExclOwnRobot();
    for(auto it = teammembers.begin(); it != teammembers.end(); it++)
    {
        auto intention = tpRTDBInputAdapter::getInstance().getIntention(it->getNumber());
        if (intention.action == actionTypeEnum::GET_BALL)
        {
            return true;
        }
    }

    return false;
}

void AbstractAction::sendIntention(const T_ACTION& actionData) const
{
    try
    {
        TRACE_FUNCTION("");

        T_INTENTION intention;
        intention.action = actionData.action;
        intention.position = actionData.position;
        tpRTDBOutputAdapter::getInstance().setIntention(intention);
    }
    catch(std::exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("AbstractAction::sendIntention Linked to: ") + e.what());
    }
}
