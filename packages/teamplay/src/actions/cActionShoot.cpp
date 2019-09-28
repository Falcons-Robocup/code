 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionShoot.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: Ivo Matthijssen
 */

#include "int/actions/cActionShoot.hpp"

#include "FalconsCommon.h"
#include "int/stores/configurationStore.hpp"
#include "int/stores/diagnosticsStore.hpp"
#include "int/cWorldStateFunctions.hpp"
#include "int/utilities/trace.hpp"
#include "int/types/cActionTypes.hpp"
#include "int/stores/fieldDimensionsStore.hpp"
#include "int/stores/robotStore.hpp"


cActionShoot::cActionShoot()
{
    boost::assign::insert( _actionParameters )
        ("shootType", std::make_pair(defaultShootTypes, false))
        ;

    _intention.action = actionTypeEnum::SHOOT;
}

cActionShoot::~cActionShoot()
{

}

// Shoot with power derived from the parameters
// Returns: PASSED    if the robot managed to shoot
//          FAILED    if something went wrong
behTreeReturnEnum cActionShoot::execute(const std::map<std::string, std::string> &parameters)
{
    behTreeReturnEnum result = behTreeReturnEnum::RUNNING;
    try
    {
        //TRACE("INFO: cActionShoot: Entered function");
        // parameter "shootingType" is the type of target we are shooting at
        std::string paramStr("shootType");

        auto paramValPair = parameters.find(paramStr);
        if (paramValPair != parameters.end())
        {
            std::string paramVal = paramValPair->second;
            shootEnum shootType = shootMapping[paramVal];

            // store diagnostics data
            auto shootTarget = getPreferredPartOfGoal();
            _intention.position.x = shootTarget.x;
            _intention.position.y = shootTarget.y;

            switch(shootType)
            {
            case shootEnum::SHOOT_TOWARDS_GOAL:
                {
                    // store diagnostics data
                    teamplay::diagnosticsStore::getDiagnostics().setAiming(true);
                    teamplay::diagnosticsStore::getDiagnostics().setShootTarget(shootTarget);

                    // Code to override a straight shot with lobshot due to limited vision range resulting in missing obstacles in front of opp goal
                    Position2D myPos = teamplay::robotStore::getInstance().getOwnRobot().getPosition();
                    Point2D oppGoalCenter = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OPP_GOALLINE_CENTER);
                    float lobShotThreshold = teamplay::configurationStore::getConfiguration().getStraightShotThreshold();

                    if (calc_distance( oppGoalCenter.x, oppGoalCenter.y, myPos.x, myPos.y) > lobShotThreshold)
                    {
                        TRACE("INFO: cActionShoot: LOB_TOWARDS_GOAL");
                        result = lobShot(shootTarget);
                    }
                    else
                    {
                        float shootHeight = 0.7; // TODO make configurable
                        TRACE("INFO: cActionShoot: SHOOT_TOWARDS_GOAL");
                        result = shoot(shootTarget.x, shootTarget.y, shootHeight);
                    }
                    break;
                }
                case shootEnum::LOB_TOWARDS_GOAL:
                {
                    // store diagnostics data
                    teamplay::diagnosticsStore::getDiagnostics().setAiming(true);
                    teamplay::diagnosticsStore::getDiagnostics().setShootTarget(shootTarget);

                    TRACE("INFO: cActionShoot: LOB_TOWARDS_GOAL");
                    result = lobShot(shootTarget);
                    break;
                }
                default:
                {
                    TRACE("Unknown shoot type encountered");
                    result = behTreeReturnEnum::FAILED;
                    break;
                }
            }
        }
        else
        {
            TRACE("WARNING: cActionShoot: no parameters given");
        }
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("cActionShoot::execute Linked to: ") + e.what());
    }

    sendIntention();
    return result;
}
