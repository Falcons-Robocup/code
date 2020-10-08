 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionDribble.cpp
 *
 *  Created on: Feb 15, 2020
 *      Author: Coen Tempelaars
 */

#include "int/actions/cActionDribble.hpp"

#include "int/stores/heightMapStore.hpp"
#include "int/stores/robotStore.hpp"
#include "cDiagnostics.hpp"


using namespace teamplay;


cActionDribble::cActionDribble()
{
    _intention.action = actionTypeEnum::MOVE;
}

cActionDribble::~cActionDribble()
{

}


/* \brief Move to a position that is suitable for shooting, lobbing or passing.
          If the current position is close enough to optimal, stay in place.
 *
 * \retval  RUNNING     if the current position is not close enough to optimal
 *          PASSED      if the current position is close enough to optimal
 *          FAILED      if something went wrong (e.g. ball not found)
 */
behTreeReturnEnum cActionDribble::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
        // Get optimal location from heightmap, using a tolerance to avoid driving long distances to reach only a slighter higher value
        float tolerance = 20.0;  // TODO: make configurable?
        auto optimum = heightMapStore::getInstance().getOptimum(CompositeHeightmapName::DRIBBLE, parameters, tolerance);

        if (positionReached(optimum.x, optimum.y))
        {
            TRACE("INFO: cActionDribble: target position reached");
            return behTreeReturnEnum::PASSED;
        }
        else /* target position not reached */
        {
            TRACE("INFO: cActionDribble: target position not reached");

            // Compute target positions
            Position2D targetPos;
            targetPos.x = optimum.x;
            targetPos.y = optimum.y;

            // Send intention
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

            // Target not reached. moveTo and return RUNNING
            moveTo(targetPos.x, targetPos.y);
            return behTreeReturnEnum::RUNNING;
        }
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("cActionDribble::execute Linked to: ") + e.what());
    }

    return behTreeReturnEnum::FAILED;
}
