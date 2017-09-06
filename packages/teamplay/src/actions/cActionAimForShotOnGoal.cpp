 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionAimForShotOnGoal.cpp
 *
 *  Created on: Feb 7, 2017
 *      Author: Ivo Matthijssen
 */

#include "int/actions/cActionAimForShotOnGoal.hpp"

#include <string>

#include "int/cTeamplayCommon.hpp"
#include "int/cWorldModelInterface.hpp"
#include "int/utilities/trace.hpp"
#include "int/cWorldStateFunctions.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/stores/diagnosticsStore.hpp"
#include "int/stores/fieldDimensionsStore.hpp"


using namespace teamplay;

cActionAimForShotOnGoal::cActionAimForShotOnGoal()
{
    boost::assign::insert( _actionParameters )
        ("distanceThreshold", std::make_pair(std::vector<std::string>{"float"}, true) )
        ("angleThreshold", std::make_pair(std::vector<std::string>{"float"}, true))
        ;
    intention.actionType = actionEnum::AIM_FOR_SHOT_ON_GOAL;
}

cActionAimForShotOnGoal::~cActionAimForShotOnGoal()
{

}

behTreeReturnEnum cActionAimForShotOnGoal::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
        // own position
        Position2D myPos;
        cWorldModelInterface::getInstance().getOwnLocation(myPos);

        // store diagnostics data
        teamplay::diagnosticsStore::getDiagnostics().setAiming(true);
        auto preferredPartOfGoal = getPreferredPartOfGoal();
        teamplay::diagnosticsStore::getDiagnostics().setShootTarget(preferredPartOfGoal);

        // calculate target position
        auto targetAngle = angle_between_two_points_0_2pi(myPos.x, myPos.y, preferredPartOfGoal.x, preferredPartOfGoal.y);
        auto targetPos = Position2D(myPos.x, myPos.y, targetAngle);

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

        // parameter "angleThreshold" is the allowed delta between the target and the real angle
        std::string angleThresholdStr("angleThreshold");
        double angleThreshold = PHIpositionTolerance;
        paramValPair = parameters.find(angleThresholdStr);
        if (paramValPair != parameters.end())
        {
            std::string angleThresholdVal = paramValPair->second;
            if (angleThresholdVal.compare(emptyValue) != 0)
            {
                angleThreshold = std::stod(angleThresholdVal);
            }
        }

        double wideAngleThreshold = angleThreshold + teamplay::configurationStore::getConfiguration().getShootTimerAngleThreshold();

        if (!positionReached(targetPos.x, targetPos.y, targetPos.phi, distanceThreshold, wideAngleThreshold))
        {
            _timer.reset();
        }

        intention.x = preferredPartOfGoal.x;
        intention.y = preferredPartOfGoal.y;
        sendIntention();

        if (positionReached(targetPos.x, targetPos.y, targetPos.phi, distanceThreshold, angleThreshold))
        {
            // Target reached. Do nothing and return PASSED
            TRACE("cActionAimForShotOnGoal PASSED (reason: target reached)");
            moveTo(myPos.x, myPos.y, myPos.phi);
            return behTreeReturnEnum::PASSED;
        }
        else
        {
            if (_timer.hasElapsed(teamplay::configurationStore::getConfiguration().getShootTimer()))
            {
                // Target not reached, but timer elapsed. Do nothing and return PASSED
                TRACE("cActionAimForShotOnGoal PASSED (reason: timer elapsed)");
                moveTo(myPos.x, myPos.y, myPos.phi);
                return behTreeReturnEnum::PASSED;
            }
            else
            {
                // Target not reached and timer not elapsed. Continue moving and return RUNNING
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
