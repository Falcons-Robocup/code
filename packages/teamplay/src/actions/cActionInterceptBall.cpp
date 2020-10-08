 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionInterceptBall.cpp
 *
 *  Created on: Jun 11, 2016
 *      Author: Jan Feitsma
 */

#include "int/actions/cActionInterceptBall.hpp"
#include "yaml-cpp/yaml.h"

#include "falconsCommon.hpp"
#include "pose2d.hpp"
#include "intersect.hpp"

#include "int/stores/ballStore.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/stores/robotStore.hpp"
#include "int/cWorldStateFunctions.hpp"
#include "cDiagnostics.hpp"

#include "boost/optional.hpp"

using namespace teamplay;

const std::string kCaptureRadiusKey = "captureRadius";

namespace hidden
{

}

cActionInterceptBall::cActionInterceptBall()
{
    _actionParameters[kCaptureRadiusKey] = std::make_pair(std::vector<std::string>{"float"}, true);
    _intention.action = actionTypeEnum::MOVE;
}

// Execute ball intercept action. If the ball is coming towards the robot, then move to intercept. Otherwise, continuously face the ball.
// Returns: PASSED    if the robot managed to get the ball
//          RUNNING   if the robot does not yet have the ball
//          FAILED    if conditions are not met
// TODO: if ball was within radius, but then moved away, should we report failure?

// parameters:
// float captureRadius = radius in meters or 'tbd': get value from teamplayInterceptBall.yaml
behTreeReturnEnum cActionInterceptBall::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
        // workaround: if other robot is dribbling with the ball, intercepting robot should not respond
        if (doesTeamHaveBall(parameters))
        {
            return behTreeReturnEnum::FAILED;
        }

        if (teamplay::robotStore::getInstance().getOwnRobot().hasBall())
        {
            TRACE("PASSED");
            return behTreeReturnEnum::PASSED;
        }

        return interceptBall("normal");

    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("cActionInterceptBall::execute Linked to: ") + e.what());
    }
    return behTreeReturnEnum::FAILED;
}

