 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionFaceNearestTeammember.cpp
 *
 *  Created on: May 4, 2016
 *      Author: Erik Kouters
 */

#include "int/actions/cActionFaceNearestTeammember.hpp"

#include "int/cTeamplayCommon.hpp"
#include "int/cWorldModelInterface.hpp"
#include "int/cWorldStateFunctions.hpp"
#include "int/utilities/trace.hpp"


cActionFaceNearestTeammember::cActionFaceNearestTeammember()
{
	intention.actionType = actionEnum::FACE_NEAREST_TEAMMEMBER;
}

cActionFaceNearestTeammember::~cActionFaceNearestTeammember()
{

}

/* \brief Stay on current position (x,y), face towards nearest teammember.
 *
 * \retval  RUNNING     if the robot is not at the target position
 *          PASSED      if the robot has reached the target position
 *          FAILED      if something went wrong
 */
behTreeReturnEnum cActionFaceNearestTeammember::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
        Position2D myPos;
        cWorldModelInterface::getInstance().getOwnLocation(myPos);

        // Nearest teammember
        Position2D nearestTeammember;
        bool foundNearestTeammember = cWorldStateFunctions::getInstance().getClosestTeammember(nearestTeammember.x, nearestTeammember.y, false);

        if (foundNearestTeammember)
        {
            double angle = angle_between_two_points_0_2pi(myPos.x, myPos.y, nearestTeammember.x, nearestTeammember.y);
            Position2D targetPos(myPos.x, myPos.y, angle);

            intention.x = targetPos.x;
            intention.y = targetPos.y;
            sendIntention();

            if (positionReached(targetPos.x, targetPos.y, targetPos.phi))
            {
                // Target reached. Do nothing and return PASSED
                TRACE("cActionFaceNearestTeammember PASSED");
                moveTo(myPos.x, myPos.y, myPos.phi);
                return behTreeReturnEnum::PASSED;
            }
            else
            {
                // Target not reached. moveTo and return RUNNING
                moveTo(targetPos.x, targetPos.y, targetPos.phi);
                return behTreeReturnEnum::RUNNING;
            }
        }
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: ") << e.what();
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return behTreeReturnEnum::FAILED;
}


