// Copyright 2018-2020 Ivo Matthijssen (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionDefendPenaltyArea.cpp
 *
 *  Created on: May 28, 2017
 *      Author: Ivo Matthijssen
 */
#include "int/actions/cActionDefendPenaltyArea.hpp"

#include "falconsCommon.hpp"

#include "int/stores/ballStore.hpp"
#include "int/stores/fieldDimensionsStore.hpp"
#include "int/stores/robotStore.hpp"
#include "cDiagnostics.hpp"

using namespace teamplay;

cActionDefendPenaltyArea::cActionDefendPenaltyArea()
{
    _intention.action = actionTypeEnum::MOVE;
}

cActionDefendPenaltyArea::~cActionDefendPenaltyArea()
{
}

// Always position between goal and ball, contrary to goalkeeper behavior
// Always face against ball
// If no ball was found, go outside penalty area in front of middle of the goal
// Returns: RUNNING always, action does not end

behTreeReturnEnum cActionDefendPenaltyArea::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
        // ball position
        ball ball = ballStore::getBall();

        // By default goto center of goal on line y referenced to outside penalty-area
        Point2D penaltyarea_left_corner = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OWN_PENALTYAREA_CORNER_LEFT);
        Point2D goalCenter = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OWN_GOALLINE_CENTER);
        Position2D targetPos = Position2D(goalCenter.x, penaltyarea_left_corner.y + _LINE_OFFSET_NORMAL, M_PI_2); // Face forward

        // if ball is known, calculate new positions for targetPos
        if (ball.isLocationKnown())
        {
            // update line y depending on how close the ball is to own goal
            if (ball.getPosition().y < _BALL_THRESHOLD_LINE_OFFSET)
            {
                targetPos.y = penaltyarea_left_corner.y + _LINE_OFFSET_CLOSE_TO_GOAL;
            }

            // projection of line the robot moves
            Vector2D leftPosRobot((fieldDimensionsStore::getFieldDimensions().getWidth() / -2.0), targetPos.y);
            Vector2D rightPosRobot((fieldDimensionsStore::getFieldDimensions().getWidth() / 2.0), targetPos.y);

            Point2D projectedGoalPost = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OWN_GOALPOST_LEFTBACK);
            if (ball.isAtLeftSide())
            {
                projectedGoalPost = fieldDimensionsStore::getFieldDimensions().getLocation(fieldPOI::OWN_GOALPOST_RIGHTBACK);
            }

            Vector2D ballPos(ball.getPosition().x, ball.getPosition().y);
            Vector2D projGoalPost(projectedGoalPost.x, projectedGoalPost.y);

            // intersect
            Vector2D intersectResult;
            if (intersect(ballPos, projGoalPost, leftPosRobot, rightPosRobot, intersectResult))
            {
                TRACE("intercept ActionDefendPenaltyArea: ") << std::to_string(intersectResult.x) << ", " << std::to_string(intersectResult.y);
                if (ball.isAtLeftSide())
                {
                    //Defend Assist moves from x=0 to the right when ball is on left side
                    targetPos.x = fmax(intersectResult.x, goalCenter.x);
                }
                else
                {
                    //Defend Assist moves from x=0 to the left when ball is on right side
                    targetPos.x = fmin(intersectResult.x, goalCenter.x);
                }
            }

        }

        _intention.position.x = targetPos.x;
        _intention.position.y = targetPos.y;
        sendIntention();

        // ensure that we move according to the rules
        if (!isCurrentPosValid())
        {
            // Move towards the center, while maintaining angle
            moveTo(0.0, 0.0);
            return behTreeReturnEnum::RUNNING;
        }

        if (!isTargetPosInsideSafetyBoundaries(targetPos))
        {
            return behTreeReturnEnum::FAILED;
        }

        //this is new

        // if move is legal, move to calculated targetPos
        moveTo(targetPos.x, targetPos.y);
        return behTreeReturnEnum::RUNNING;
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("cActionDefendPenaltyArea::execute Linked to: ") + e.what());
    }

    return behTreeReturnEnum::FAILED;
}
