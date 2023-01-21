// Copyright 2019-2022 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * CheckTargetValid.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#include "int/PathPlanningAlgorithms.hpp"

#include "cEnvironmentField.hpp"
#include "cDiagnostics.hpp"


// determine what to do
void handleViolation(PathPlanningData &data, BoundaryOptionEnum mode)
{
    switch (mode)
    {
        case BoundaryOptionEnum::STOP_AND_PASS:
            data.done = true;
            data.stop = true;
            data.resultStatus = actionResultTypeEnum::PASSED;
            break;
        case BoundaryOptionEnum::STOP_AND_FAIL:
            data.done = true;
            data.stop = true;
            data.resultStatus = actionResultTypeEnum::FAILED;
            break;
        case BoundaryOptionEnum::CLIP:
            ; // falling through, do nothing (not in this function anyway)
        case BoundaryOptionEnum::ALLOW:
            ; // falling through, do nothing
        default:
            ; // do nothing
    }
}

void CheckTargetValid::execute(PathPlanningData &data)
{
    TRACE_FUNCTION("");
    Position2D target = data.getSubTarget();

    // check option: what to do if target is inside a forbidden area
    auto mode = data.configPP.boundaries.targetInsideForbiddenArea;
    if (mode != BoundaryOptionEnum::ALLOW)
    {
        for (auto it = data.forbiddenAreas.begin(); it != data.forbiddenAreas.end(); ++it)
        {
            if (it->isPointInside(vec2d(target.x, target.y)))
            {
                // JPGL - 2022-06-27
                //      This trace was converted to an event log warning in order to
                //      monitor how often we need to draw a forbidden area on the ball
                //      We might want to consider putting this back to TRACE if it spams
                //      th event logs too much
                TRACE_WARNING("target is in forbidden area id=%d", it->id);
                if (mode == BoundaryOptionEnum::CLIP)
                {
                    // TODO: implement clipping option (geometry)
                    TRACE_WARNING("target clipping to forbidden area is not yet implemented");
                    mode = BoundaryOptionEnum::STOP_AND_FAIL;
                }
                // handle the violation
                handleViolation(data, mode);
            }
        }
    }

    // allow target position in TTA for park/substitute
    bool targetInsideTTA = false;
    if (cEnvironmentField::getInstance().isPositionInArea(target.x, target.y, A_TTA))
    {
        targetInsideTTA = true;
    }

    // check option: what to do if target is outside of field
    mode = data.configPP.boundaries.targetOutsideField;
    if (mode != BoundaryOptionEnum::ALLOW)
    {
        float fieldMarginX = data.configPP.boundaries.fieldMarginX;
        float fieldMarginY = data.configPP.boundaries.fieldMarginY;
        float fieldLength = cEnvironmentField::getInstance().getLength();
        float fieldWidth = cEnvironmentField::getInstance().getWidth();
        float limit = fieldWidth * 0.5 + fieldMarginX;
        if (fabs(target.x) > limit && !targetInsideTTA)
        {
            TRACE("target field dimension X violation (limit=%6.2f)", limit);
            handleViolation(data, mode);
            if (mode == BoundaryOptionEnum::CLIP)
            {
                float tmp = target.x;
                clip(tmp, 0.0f, limit);
                data.path[0].pos.x = tmp;
            }
        }
        limit = fieldLength * 0.5 + fieldMarginY;
        if (fabs(target.y) > limit && !targetInsideTTA)
        {
            TRACE("target field dimension Y violation (limit=%6.2f)", limit);
            handleViolation(data, mode);
            if (mode == BoundaryOptionEnum::CLIP)
            {
                float tmp = target.y;
                clip(tmp, 0.0f, limit);
                data.path[0].pos.y = tmp;
            }
        }
    }

    // check options: own and opponent halves
    // (especially the CLIP options are useful for playing on half a demo field)
    mode = data.configPP.boundaries.targetOnOwnHalf;
    if (mode != BoundaryOptionEnum::ALLOW)
    {
        if (target.y < 0.0)
        {
            TRACE("target on own half not allowed");
            handleViolation(data, mode);
            if (mode == BoundaryOptionEnum::CLIP)
            {
                data.path[0].pos.y = 0.0;
            }
        }
    }
    mode = data.configPP.boundaries.targetOnOpponentHalf;
    if (mode != BoundaryOptionEnum::ALLOW)
    {
        if (target.y > 0.0)
        {
            TRACE("target on opponent half not allowed");
            handleViolation(data, mode);
            if (mode == BoundaryOptionEnum::CLIP)
            {
                data.path[0].pos.y = 0.0;
            }
        }
    }
}

