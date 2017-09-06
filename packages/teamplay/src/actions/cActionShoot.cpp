 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
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

#include "int/stores/configurationStore.hpp"
#include "int/stores/diagnosticsStore.hpp"
#include "int/cTeamplayCommon.hpp"
#include "int/cWorldModelInterface.hpp"
#include "int/cWorldStateFunctions.hpp"
#include "int/utilities/trace.hpp"
#include "int/types/cActionTypes.hpp"
#include "int/stores/fieldDimensionsStore.hpp"

cActionShoot::cActionShoot()
{
    boost::assign::insert( _actionParameters )
        ("shootType", std::make_pair(defaultShootTypes, false))
        ;

    intention.actionType = actionEnum::SHOOT;
}

cActionShoot::~cActionShoot()
{

}

// Shoot with power derived from the parameters
// Returns: PASSED    if the robot managed to shoot
//          FAILED    if something went wrong
behTreeReturnEnum cActionShoot::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
        //TRACE("INFO: cActionShoot: Entered function");
        // parameter "shootingType" is the type of target we are shooting at
        std::string paramStr("shootType");

        auto paramValPair = parameters.find(paramStr);
        // before shooting check if we have the ball to avoid damaging the shooterbox

        ballPossession_struct_t ballPossession;
        cWorldModelInterface::getInstance().getBallPossession(ballPossession);
        if(ballPossession.possessionType == ballPossessionEnum::TEAMMEMBER && ballPossession.robotID == cWorldStateFunctions::getInstance().getRobotID())
        {
            if (paramValPair != parameters.end())
            {
                Position2D closestTeammember;
                Position2D myPos;

                std::string paramVal = paramValPair->second;
                shootEnum shootType = shootMapping[paramVal];

                switch(shootType)
                {
                case shootEnum::PASS_TOWARDS_NEAREST_TEAMMEMBER:
                    {
                        cWorldModelInterface::getInstance().getOwnLocation(myPos);
                        cWorldStateFunctions::getInstance().getClosestTeammember(closestTeammember.x, closestTeammember.y, false);

                        TRACE("INFO: cActionShoot: PASS_TOWARDS_NEAREST_TEAMMEMBER");
                        shoot(getPassPower(calc_distance(myPos, closestTeammember)));
                        break;
                    }

                    case shootEnum::PASS_TOWARDS_NEAREST_ATTACKER:
                    {
                        cWorldModelInterface::getInstance().getOwnLocation(myPos);
                        cWorldStateFunctions::getInstance().getClosestAttacker(A_FIELD, closestTeammember.x, closestTeammember.y);

                        TRACE("INFO: cActionShoot: PASS_TOWARDS_NEAREST_ATTACKER");
                        shoot(getPassPower(calc_distance(myPos, closestTeammember)));
                        break;
                    }

                    case shootEnum::PASS_TOWARDS_FURTHEST_ATTACKER:
                    {
                        cWorldModelInterface::getInstance().getOwnLocation(myPos);
                        cWorldStateFunctions::getInstance().getClosestAttackerToOpponentGoal(closestTeammember.x, closestTeammember.y);

                        TRACE("INFO: cActionShoot: PASS_TOWARDS_FURTHEST_ATTACKER");
                        shoot(getPassPower(calc_distance(myPos, closestTeammember)));
                        break;
                    }

                    case shootEnum::PASS_TOWARDS_NEAREST_ATTACKER_ON_OPP_HALF:
                    {
                        cWorldModelInterface::getInstance().getOwnLocation(myPos);
                        cWorldStateFunctions::getInstance().getClosestAttacker(A_OPP_SIDE, closestTeammember.x, closestTeammember.y);

                        TRACE("INFO: cActionShoot: PASS_TOWARDS_NEAREST_ATTACKER_ON_OPP_HALF");
                        shoot(getPassPower(calc_distance(myPos, closestTeammember)));
                        break;
                    }

                    case shootEnum::PASS_TOWARDS_TIP_IN_POSITION:
                    {
                        cWorldModelInterface::getInstance().getOwnLocation(myPos);
                        Point2D tipInPOI = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::TIP_IN);
                        Position2D tipInPOI_Pos2D;
                        tipInPOI_Pos2D.x = tipInPOI.x;
                        tipInPOI_Pos2D.y = tipInPOI.y;

                        TRACE("INFO: cActionShoot: PASS_TOWARDS_TIP_IN_POSITION");
                        shoot(getPassPower(calc_distance(myPos, tipInPOI_Pos2D)));
                        break;
                    }

                    case shootEnum::SHOOT_TOWARDS_GOAL:
                    {
                        // store diagnostics data
                        teamplay::diagnosticsStore::getDiagnostics().setAiming(true);
                        teamplay::diagnosticsStore::getDiagnostics().setShootTarget(getPreferredPartOfGoal());

                        TRACE("INFO: cActionShoot: SHOOT_TOWARDS_GOAL");
                        shoot(teamplay::configurationStore::getConfiguration().getShootAtGoalPower());
                        break;
                    }
                    case shootEnum::LOB_TOWARDS_GOAL:
                    {
                        // store diagnostics data
                        teamplay::diagnosticsStore::getDiagnostics().setAiming(true);
                        teamplay::diagnosticsStore::getDiagnostics().setShootTarget(getPreferredPartOfGoal());

                        TRACE("INFO: cActionShoot: LOB_TOWARDS_GOAL");
                        lobShot(getPreferredPartOfGoal());
                        break;
                    }
                    case shootEnum::PASS_TOWARDS_GOALIE:
                    {
                        cWorldModelInterface::getInstance().getOwnLocation(myPos);
                        boost::optional<robotNumber> goalie = cWorldStateFunctions::getInstance().getRobotWithRole(treeEnum::R_GOALKEEPER);
                        if (goalie)
                        {
                            Position2D goalieLocation = cWorldStateFunctions::getInstance().getLocationOfRobot(*goalie);
                            TRACE("INFO: cActionShoot: PASS_TOWARDS_GOALIE");
                            shoot(getPassPower(calc_distance(myPos, goalieLocation)));
                        }
                        else
                        {
                            TRACE("WARNING: cActionShoot: PASS_TOWARDS_GOALIE while robot is goalie. No action.");
                            return behTreeReturnEnum::FAILED;
                        }
                        break;
                    }
                    default:
                    {
                        TRACE("Unknown shoot type encountered");
                        break;
                    }
                }
            }
            else
            {
            	TRACE("WARNING: cActionShoot: no parameters given");
            }
        }
        else
        {
            TRACE("WARNING: cActionShoot: tried to shoot without ball");
        }
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: ") << e.what();
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    sendIntention();
    return behTreeReturnEnum::PASSED;
}


float cActionShoot::getPassPower (teamplay::passDistance distance)
{
    auto pass_powers = teamplay::configurationStore::getConfiguration().getPassPowers();
    return pass_powers.find(distance)->second;
}

float cActionShoot::getPassPower (float distance)
{
    if (distance < 0.0)
    {
        TRACE("Minor violation: attempt to pass a negative distance.");
        distance = -distance;
    }

    auto pass_ranges = teamplay::configurationStore::getConfiguration().getPassRanges();
    for (auto it = pass_ranges.begin(); it != pass_ranges.end(); it++)
    {
        if (distance < it->second)
        {
            return getPassPower(it->first);
        }
    }

    return getPassPower(teamplay::passDistance::LONG);
}
