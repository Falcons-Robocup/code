 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cActionPass.cpp
 *
 *  Created on: Jan 04, 2018
 *      Author: Ivo Matthijssen
 */

#include "int/actions/cActionPass.hpp"

#include "FalconsCommon.h"
#include "int/stores/configurationStore.hpp"
#include "int/stores/diagnosticsStore.hpp"
#include "int/cWorldStateFunctions.hpp"
#include "int/utilities/trace.hpp"
#include "int/types/cActionTypes.hpp"
#include "int/stores/fieldDimensionsStore.hpp"
#include "int/stores/robotStore.hpp"


cActionPass::cActionPass()
{
    boost::assign::insert( _actionParameters )
        ("passType", std::make_pair(defaultPassTypes, false))
        ;

    _intention.action = actionTypeEnum::PASS;
}

cActionPass::~cActionPass()
{

}

// Returns: PASSED    if the robot managed to pass
//            RUNNING      if the robot is busy trying to pass
//          FAILED    if something went wrong
behTreeReturnEnum cActionPass::execute(const std::map<std::string, std::string> &parameters)
{
    behTreeReturnEnum result = behTreeReturnEnum::RUNNING;
    try
    {
        //TRACE("INFO: cActionShoot: Entered function");
        // parameter "shootingType" is the type of target we are shooting at
        std::string paramStr("passType");

        auto paramValPair = parameters.find(paramStr);
        if (paramValPair != parameters.end())
        {
            Position2D teammember;

            std::string paramVal = paramValPair->second;
            shootEnum passType = shootMapping[paramVal];

            switch(passType)
            {
            case shootEnum::PASS_TOWARDS_NEAREST_TEAMMEMBER:
                {
                    cWorldStateFunctions::getInstance().getClosestTeammember(teammember.x, teammember.y, false);

                    TRACE("INFO: cActionPass: PASS_TOWARDS_NEAREST_TEAMMEMBER");
                    result = pass(teammember.x, teammember.y);
                    _intention.position.x = teammember.x;
                    _intention.position.y = teammember.y;
                    break;
                }

                case shootEnum::PASS_TOWARDS_NEAREST_ATTACKER:
                {
                    cWorldStateFunctions::getInstance().getClosestAttacker(A_FIELD, teammember.x, teammember.y);

                    TRACE("INFO: cActionPass: PASS_TOWARDS_NEAREST_ATTACKER");
                    result = pass(teammember.x, teammember.y);
                    _intention.position.x = teammember.x;
                    _intention.position.y = teammember.y;
                    break;
                }

                case shootEnum::PASS_TOWARDS_FURTHEST_ATTACKER:
                {
                    cWorldStateFunctions::getInstance().getClosestAttackerToOpponentGoal(teammember.x, teammember.y);

                    TRACE("INFO: cActionPass: PASS_TOWARDS_FURTHEST_ATTACKER");
                    result = pass(teammember.x, teammember.y);
                    _intention.position.x = teammember.x;
                    _intention.position.y = teammember.y;
                    break;
                }

                case shootEnum::PASS_TOWARDS_NEAREST_ATTACKER_ON_OPP_HALF:
                {
                    cWorldStateFunctions::getInstance().getClosestAttacker(A_OPP_SIDE, teammember.x, teammember.y);

                    TRACE("INFO: cActionPass: PASS_TOWARDS_NEAREST_ATTACKER_ON_OPP_HALF");
                    result = pass(teammember.x, teammember.y);
                    _intention.position.x = teammember.x;
                    _intention.position.y = teammember.y;
                    break;
                }

                case shootEnum::PASS_TOWARDS_FURTHEST_DEFENDER:
                {
                    cWorldStateFunctions::getInstance().getClosestDefenderToOpponentGoal(teammember.x, teammember.y);

                    TRACE("INFO: cActionPass: PASS_TOWARDS_FURTHEST_DEFENDER");
                    result = pass(teammember.x, teammember.y);
                    _intention.position.x = teammember.x;
                    _intention.position.y = teammember.y;
                    break;
                }

                case shootEnum::PASS_TOWARDS_TIP_IN_POSITION:
                {
                    Point2D tipInPOI = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::TIP_IN);

                    TRACE("INFO: cActionPass: PASS_TOWARDS_TIP_IN_POSITION");
                    result = pass(tipInPOI.x, tipInPOI.y);
                    _intention.position.x = tipInPOI.x;
                    _intention.position.y = tipInPOI.y;
                    break;
                }

                default:
                {
                    TRACE("Unknown pass type encountered");
                    result = behTreeReturnEnum::FAILED;
                    break;
                }
            }
        }
        else
        {
            TRACE("WARNING: cActionPass: no parameters given");
        }
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("cActionPass::execute Linked to: ") + e.what());
    }

    sendIntention();
    return result;
}
