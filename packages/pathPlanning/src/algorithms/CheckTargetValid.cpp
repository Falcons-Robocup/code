 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * CheckTargetValid.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#include "int/PathPlanningAlgorithms.hpp"
#include "int/facilities/clipping.hpp"

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
    auto mode = data.config.boundaries.targetInsideForbiddenArea;
    if (mode != BoundaryOptionEnum::ALLOW)
    {
        for (auto it = data.forbiddenAreas.begin(); it != data.forbiddenAreas.end(); ++it)
        {
            if (it->isPointInside(vec2d(target.x, target.y)))
            {
                TRACE("target is in forbidden area id=%d", it->id);
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
    mode = data.config.boundaries.targetOutsideField;
    if (mode != BoundaryOptionEnum::ALLOW)
    {
        float fieldMarginX = data.config.boundaries.fieldMarginX;
        float fieldMarginY = data.config.boundaries.fieldMarginY;
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
    mode = data.config.boundaries.targetOnOwnHalf;
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
    mode = data.config.boundaries.targetOnOpponentHalf;
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

