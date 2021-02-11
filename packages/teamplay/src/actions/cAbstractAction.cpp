// Copyright 2016-2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cAbstractAction.cpp
 *
 *  Created on: Apr 30, 2016
 *      Author: Erik Kouters
 */

#include "int/actions/cAbstractAction.hpp"

#include <boost/lexical_cast.hpp>
#include <stdexcept>

#include "falconsCommon.hpp"
#include "int/cWorldStateFunctions.hpp"
#include "int/rules/ruleAvoidAreas.hpp"
#include "int/stores/ballStore.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/stores/fieldDimensionsStore.hpp"
#include "int/stores/robotStore.hpp"

#include "int/adapters/cRTDBInputAdapter.hpp"
#include "int/adapters/cRTDBOutputAdapter.hpp"

#include "tracing.hpp"

using namespace teamplay;

/* Static definitions */
static boost::shared_ptr<ruleAvoidAreas> rule_p;
static boost::shared_ptr<timer> timer_p;


cAbstractAction::cAbstractAction()
{
    _intention.action = actionTypeEnum::UNKNOWN;
    _actionId = 0;

    timer_p.reset(new timer());
    rule_p.reset(new ruleAvoidAreas(timer_p));
}

cAbstractAction::~cAbstractAction()
{
    rule_p.reset();
    timer_p.reset();
}

behTreeReturnEnum cAbstractAction::execute(const std::map<std::string, std::string> &parameters)
{
    throw std::runtime_error("Not implemented");
}

/* \brief Parses string to get Position2D from POI, ball or robotPos.
 *
 */
boost::optional<Position2D> cAbstractAction::getPos2DFromStr(const std::map<std::string, std::string> &parameters, std::string &param)
{
    Position2D retVal;

    auto paramVal = parameters.find(param);
    if (paramVal != parameters.end())
    {
        std::string strParam = paramVal->second;
        return cWorldStateFunctions::getInstance().getPositionOfPOI(strParam);
    }

    return boost::none;
}

/* \brief Parses string to get Position2D from POI, ball or robotPos.
 *
 */
boost::optional<Area2D> cAbstractAction::getArea2DFromStr(const std::map<std::string, std::string> &parameters, std::string &param)
{
    Area2D retVal;

    auto paramVal = parameters.find(param);
    if (paramVal != parameters.end())
    {
        std::string strParam = paramVal->second;

        try
        {
            return fieldDimensionsStore::getFieldDimensions().getArea(strParam);
        }
        catch (std::exception &e)
        {

            return boost::none;
        }
    }
    return boost::none;
}

T_ACTION cAbstractAction::makeAction()
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

void cAbstractAction::setForbiddenAreas()
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
    cRTDBOutputAdapter::getInstance().setForbiddenAreas(forbiddenAreas);
}

behTreeReturnEnum cAbstractAction::translateActionResultToTreeEnum(T_ACTION_RESULT actionResult)
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

behTreeReturnEnum cAbstractAction::moveTo( double x, double y, const std::string& motionProfile )
{
    std::ostringstream ss;
    ss << "target: (" << x << ", " << y << ")";
    TRACE_FUNCTION(ss.str().c_str());

    std::map<std::string, std::string> params;
    bool ballHandlersEnabled = true;
    if (isPrepareSetPiece(params) || isParkingSetPiece(params))
    {
        ballHandlersEnabled = false;
    }

    Position2D myPos = robotStore::getInstance().getOwnRobot().getPosition();
    Position2D pose(x, y, myPos.phi);

    // If we do not have the ball: Always face ball.
    auto robot = robotStore::getInstance().getOwnRobot();
    if (!robot.hasBall())
    {
        auto ball = ballStore::getBall();
        if (ball.isLocationKnown())
        {
            auto ballPos = ballStore::getBall().getPosition();
            pose.phi = angle_between_two_points_0_2pi(myPos.x, myPos.y, ballPos.x, ballPos.y);
        }
    }

    setForbiddenAreas();
    
    T_ACTION actionData = makeAction();
    actionData.action = actionTypeEnum::MOVE;
    actionData.position.x = pose.x;
    actionData.position.y = pose.y;
    actionData.position.z = pose.phi;
    actionData.motionType = determineMotionType(motionProfile);
    actionData.ballHandlersEnabled = ballHandlersEnabled;
    return executeAction(actionData);
}

behTreeReturnEnum cAbstractAction::pass(float x, float y)
{
    std::ostringstream ss;
    ss << "target: (" << x << ", " << y << ")";
    TRACE_FUNCTION(ss.str().c_str());

    T_ACTION actionData = makeAction();
    actionData.action = actionTypeEnum::PASS;
    actionData.position.x = x;
    actionData.position.y = y;

    return executeAction(actionData);
}

behTreeReturnEnum cAbstractAction::shoot(const Point2D& target)
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
    return executeAction(actionData);
}

behTreeReturnEnum cAbstractAction::lobShot(const Point2D& target)
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
    return executeAction(actionData);
}

behTreeReturnEnum cAbstractAction::getBall(const std::string& motionProfile)
{
    TRACE_FUNCTION("");

    setForbiddenAreas();

    T_ACTION actionData = makeAction();
    actionData.action = actionTypeEnum::GET_BALL;
    actionData.motionType = determineMotionType(motionProfile);
    return executeAction(actionData);
}

behTreeReturnEnum cAbstractAction::turnAwayFromOpponent(double x, double y)
{
    std::ostringstream ss;
    ss << "target: (" << x << ", " << y << ")";
    TRACE_FUNCTION(ss.str().c_str());

    T_ACTION actionData = makeAction();
    actionData.action = actionTypeEnum::TURN_AWAY_FROM_OPPONENT;
    actionData.position.x = x;
    actionData.position.y = y;
    return executeAction(actionData);
}

behTreeReturnEnum cAbstractAction::keeperMove(float x)
{
    std::ostringstream ss;
    ss << "target: (x=" << x << ")";
    TRACE_FUNCTION(ss.str().c_str());

    T_ACTION actionData = makeAction();
    actionData.action = actionTypeEnum::KEEPER_MOVE;
    actionData.position.x = x;
    return executeAction(actionData);
}

behTreeReturnEnum cAbstractAction::interceptBall()
{
    TRACE_FUNCTION("");

    T_ACTION actionData = makeAction();
    actionData.action = actionTypeEnum::INTERCEPT_BALL;
    return executeAction(actionData);
}

void cAbstractAction::stop()
{
    /* The "stop" must (1) move to current position and (2) disable motion and the HAL
    * Point (1) is important, otherwise motion will continue pursuing its current setpoint
    * until the setpoint watchdog interferes 
    * 
    * These details are now moved to motionPlanning.
    */

    T_ACTION actionData = makeAction();
    actionData.action = actionTypeEnum::STOP;
    executeAction(actionData); // ignore return value
}

bool cAbstractAction::positionReached(double x, double y)
{
    return positionReached(x, y, XYpositionTolerance);
}

bool cAbstractAction::positionReached(double x, double y, double xy_threshold)
{
    Position2D myPos = robotStore::getInstance().getOwnRobot().getPosition();

    TRACE(" x: ") << std::to_string(x)
        << " y: " << std::to_string(y)
        << " xy threshold: " << std::to_string(xy_threshold);

    bool positionReached = ((fabs(myPos.x - x) < xy_threshold) && (fabs(myPos.y - y) < xy_threshold));

    TRACE("position reached: ") << ((positionReached) ? ("yes") : ("no"));

    return positionReached;
}

bool cAbstractAction::isCurrentPosValid() const
{
    return rule_p->isCurrentPositionValid();
}

bool cAbstractAction::isTargetPosInsideSafetyBoundaries (const Position2D& targetPos) const
{
    return fieldDimensionsStore::getFieldDimensions().isPositionInSafetyBoundaries(targetPos.x, targetPos.y);
}

std::vector<polygon2D> cAbstractAction::getForbiddenAreas() const
{
    TRACE_FUNCTION("");
    auto forbiddenAreas = rule_p->getForbiddenAreas();
    TRACE("FBA: got %d forbiddenAreas from rules", (int)forbiddenAreas.size());

    if (ballStore::getBall().mustBeAvoided())
    {
        auto ballPosition = ballStore::getBall().getPosition();
        polygon2D squareAroundBall;
        squareAroundBall.addPoint(ballPosition.x - 0.25, ballPosition.y - 0.25);
        squareAroundBall.addPoint(ballPosition.x - 0.25, ballPosition.y + 0.25);
        squareAroundBall.addPoint(ballPosition.x + 0.25, ballPosition.y + 0.25);
        squareAroundBall.addPoint(ballPosition.x + 0.25, ballPosition.y - 0.25);


        forbiddenAreas.push_back(squareAroundBall);
        TRACE("FBA: avoiding ball as well");
    }

    /* Check whether other robot is shooting on goal
    * If so, add forbidden area on those coordinates
    */
    std::vector<polygon2D> forbiddenActionAreas = getForbiddenActionAreas();
    TRACE("FBA: avoiding %d action areas", (int)forbiddenActionAreas.size());

    forbiddenAreas.insert(forbiddenAreas.begin(), forbiddenActionAreas.begin(), forbiddenActionAreas.end());

    TRACE("FBA: total %d forbiddenAreas", (int)forbiddenAreas.size());
    return forbiddenAreas;
}


Point2D cAbstractAction::getPreferredPartOfGoal() const
{
    // own position
    Position2D myPos = robotStore::getInstance().getOwnRobot().getPosition();

    // some variables that read nice
    auto goalCenter = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OPP_GOALLINE_CENTER);
    auto goalPostLeft = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OPP_GOALPOST_LEFT);
    auto goalPostRight = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OPP_GOALPOST_RIGHT);

    // the robot should shoot inside the goal area i.o. onto the goalpost
    // shoot target is relative to goal width, so robust for legacy and new (2018) field size

    // below code should be identical to cWorldStateFunctions getPreferredShootXYOfGoal
    // TODO: find common location for this function
    float goalPostShootOffset = 0.375; // the robot should shoot inside the goal area i.o. onto the goalpost
    auto targetLeft = goalPostLeft;
    auto targetRight = goalPostRight;
    targetLeft.x = goalPostLeft.x + goalPostShootOffset;
    targetRight.x = goalPostRight.x - goalPostShootOffset;
    
    // Shoot on x=0 when far away from opponent goal (shoot accuracy)
    float distanceThreshold = teamplay::configurationStore::getConfiguration().getAimForCornerThreshold();
    float distanceToGoal = (goalCenter - Point2D(myPos.x, myPos.y)).size();
    if (distanceToGoal > distanceThreshold)
    {
        // far away -> override to aim at center of goal
        targetLeft.x = 0.0;
        targetRight.x = 0.0;
    }

    // default aim is opponent goalline center
    auto preferredPartOfGoal = goalCenter;

    std::map<std::string, std::string> params;
    if ( isOpponentGoalKeeperInLeftCorner(params) )
    {
        // Opponent goalkeeper is positioned in left corner, shoot in right corner
        preferredPartOfGoal = targetRight;
    }
    else if ( isOpponentGoalKeeperInRightCorner(params) )
    {
        // Opponent goalkeeper is positioned in right corner, shoot in left corner
        preferredPartOfGoal = targetLeft;
    }
    else
    {
        // is the robot positioned on left side of field, shoot right corner
        if (fieldDimensionsStore::getFieldDimensions().isPositionInLeftSide(myPos.x, myPos.y))
        {
            preferredPartOfGoal = targetRight;
        }
        else
        {
            // else shoot in left corner
            preferredPartOfGoal = targetLeft;
        }
    }

    // CONTAINMENT MATCH VDL
    // ALWAYS SHOOT AT X=0
    preferredPartOfGoal = goalCenter;

    return preferredPartOfGoal;
}

void cAbstractAction::sendIntention()
{
    try
    {
        TRACE_FUNCTION("");
        if (_intention.action != actionTypeEnum::UNKNOWN)
        {
            cRTDBOutputAdapter::getInstance().setIntention(_intention);
        }
    }
    catch(std::exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("cAbstractAction::sendIntention Linked to: ") + e.what());
    }
}

std::vector<polygon2D> cAbstractAction::getForbiddenActionAreas() const
{
    TRACE_FUNCTION("");
    std::vector<polygon2D> retVal;
    // Check shooting on goal
    auto teammembers = robotStore::getInstance().getAllRobotsExclOwnRobot();
    for(auto it = teammembers.begin(); it != teammembers.end(); it++)
    {
        auto intention = cRTDBInputAdapter::getInstance().getIntention(it->getNumber());
        if (intention.action == actionTypeEnum::SHOOT)
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

behTreeReturnEnum cAbstractAction::executeAction(T_ACTION const &actionData)
{
    // wrapper to call motionPlanning and log diagnostics data
    cRTDBOutputAdapter::getInstance().setActionData(actionData); // diagnostics, no waitForPut anymore
    return translateActionResultToTreeEnum( cRTDBOutputAdapter::getInstance().getMPClient().executeAction(actionData) );
}


motionTypeEnum cAbstractAction::determineMotionType(const std::string& motionProfile)
{
    motionTypeEnum motionType = motionTypeEnum::INVALID;
    if (motionProfile != "normal")
    {
        motionType = motionTypeEnum::ACCURATE;
    }
    else
    {
        motionType = motionTypeEnum::NORMAL;
    }

    return motionType;
}