// Copyright 2015-2020 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cActionShoot.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: Ivo Matthijssen
 */

#include "int/actions/cActionShoot.hpp"

#include "falconsCommon.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/stores/diagnosticsStore.hpp"
#include "int/cWorldStateFunctions.hpp"
#include "cDiagnostics.hpp"
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
                        _intention.action = actionTypeEnum::LOB;
                        result = lobShot(shootTarget);
                    }
                    else
                    {
                        TRACE("INFO: cActionShoot: SHOOT_TOWARDS_GOAL");
                        _intention.action = actionTypeEnum::SHOOT;
                        result = shoot(shootTarget);
                    }
                    break;
                }
                case shootEnum::LOB_TOWARDS_GOAL:
                {
                    // store diagnostics data
                    teamplay::diagnosticsStore::getDiagnostics().setAiming(true);
                    teamplay::diagnosticsStore::getDiagnostics().setShootTarget(shootTarget);

                    TRACE("INFO: cActionShoot: LOB_TOWARDS_GOAL");
                    _intention.action = actionTypeEnum::LOB;
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
