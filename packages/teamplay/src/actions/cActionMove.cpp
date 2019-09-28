 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionMove.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: Ivo Matthijssen
 */

#include "int/actions/cActionMove.hpp"

#include <string>

#include "FalconsCommon.h"
#include "int/stores/ballStore.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/stores/robotStore.hpp"
#include "int/utilities/trace.hpp"


using namespace teamplay;

cActionMove::cActionMove()
{
    boost::assign::insert( _actionParameters )
        ("target", std::make_pair(defaultPOI, false) )
        ("distanceThreshold", std::make_pair(std::vector<std::string>{"float"}, true) )
        ("motionProfile", std::make_pair(std::vector<std::string>{defaultMotionProfiles}, true))
        ;

    _intention.action = actionTypeEnum::MOVE;
}

cActionMove::~cActionMove()
{

}

/* \brief Move to 'target' (x,y)
 *
 * \param target              The target (x,y)
 * \param distanceThreshold   [Optional] The allowed delta between the target position and current position
 *
 * \retval  RUNNING     if the robot is not at the target position
 *          PASSED      if the robot has reached the target position
 *          FAILED      if something went wrong
 */
behTreeReturnEnum cActionMove::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
        Position2D myPos = robotStore::getInstance().getOwnRobot().getPosition();

        // parameter "target" is the place to go.
        std::string targetStr("target");
        boost::optional<Position2D> target = getPos2DFromStr(parameters, targetStr);

        // parameter "distanceThreshold" is the allowed delta between the target and the real position
        std::string distanceThresholdStr("distanceThreshold");
        double distanceThreshold = XYpositionTolerance;
        auto paramValPair = parameters.find(distanceThresholdStr);
        if (paramValPair != parameters.end())
        {
            std::string distanceThresholdVal = paramValPair->second;
            if (distanceThresholdVal.compare(emptyValue) != 0)
            {
                distanceThreshold = std::stod(distanceThresholdVal);
            }
        }

        // [Optional] parameter "motionProfile" defines with which profile to move with (e.g., normal play or more careful during a setpiece)
        std::string motionProfileStr("motionProfile");
        std::string motionProfileValue = "normal";
        paramValPair = parameters.find(motionProfileStr);
        if (paramValPair != parameters.end())
        {
            motionProfileValue = paramValPair->second;
        }

        if (target)
        {
            Position2D targetPos = *target;

            _intention.position.x = targetPos.x;
            _intention.position.y = targetPos.y;
            sendIntention();

            // ensure that we move according to the rules
            if (!isCurrentPosValid())
            {
                // Move towards the center, while maintaining angle and motion profile
                moveTo(0.0, 0.0, motionProfileValue);
                return behTreeReturnEnum::RUNNING;
            }

            if (!isTargetPosInsideSafetyBoundaries(targetPos))
            {
                return behTreeReturnEnum::FAILED;
            }

            // Previously determined targetPos.
            // Now move there, if not already reached.
            if (positionReached(targetPos.x, targetPos.y, distanceThreshold))
            {
                // Target reached. Do nothing and return PASSED
                TRACE("cActionMove PASSED (reason: target reached)");
                moveTo(myPos.x, myPos.y);
                return behTreeReturnEnum::PASSED;
            }
            else
            {
                //moveTo and return RUNNING
                moveTo(targetPos.x, targetPos.y, motionProfileValue);
                return behTreeReturnEnum::RUNNING;
            }
        }

    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("cActionMove::execute Linked to: ") + e.what());
    }

    return behTreeReturnEnum::FAILED;
}
