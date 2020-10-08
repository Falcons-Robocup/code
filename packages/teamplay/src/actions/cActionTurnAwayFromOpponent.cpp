 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionTurnAwayFromOpponent.cpp
 *
 *  Created on: Apr 28, 2018
 *      Author: Erik Kouters
 */

#include "int/actions/cActionTurnAwayFromOpponent.hpp"

#include <string>

#include "falconsCommon.hpp"
#include "int/stores/ballStore.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/stores/robotStore.hpp"
#include "cDiagnostics.hpp"


using namespace teamplay;

cActionTurnAwayFromOpponent::cActionTurnAwayFromOpponent()
{
    boost::assign::insert( _actionParameters )
        ("target", std::make_pair(defaultPOI, false) )
        ("motionProfile", std::make_pair(std::vector<std::string>{defaultMotionProfiles}, true))
        ;

    _intention.action = actionTypeEnum::MOVE;
}

cActionTurnAwayFromOpponent::~cActionTurnAwayFromOpponent()
{

}

/* \brief Turn away from 'target' (x,y)
 *
 * \param target              The target (x,y)
 *
 * \retval  RUNNING     if the robot is not at the target position
 *          PASSED      if the robot has reached the target position
 *          FAILED      if something went wrong
 */
behTreeReturnEnum cActionTurnAwayFromOpponent::execute(const std::map<std::string, std::string> &parameters)
{
    behTreeReturnEnum result = behTreeReturnEnum::RUNNING;
    try
    {
        // parameter "target" is the place to go.
        std::string targetStr("target");
        boost::optional<Position2D> target = getPos2DFromStr(parameters, targetStr);

        // [Optional] parameter "motionProfile" defines with which profile to move with (e.g., normal play or more careful during a setpiece)
        std::string motionProfileStr("motionProfile");
        std::string motionProfileValue = "normal";
        auto paramValPair = parameters.find(motionProfileStr);
        if (paramValPair != parameters.end())
        {
            motionProfileValue = paramValPair->second;
        }

        if (target)
        {
            Position2D targetPos = *target;

            // Send in the intention the (x,y) of the opponent we are looking away from
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

            result = turnAwayFromOpponent(targetPos.x, targetPos.y, motionProfileValue);
            return result;
        }

    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("cActionTurnAwayFromOpponent::execute Linked to: ") + e.what());
    }

    return behTreeReturnEnum::FAILED;
}
