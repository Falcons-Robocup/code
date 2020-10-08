 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
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

#include "falconsCommon.hpp"
#include "intersect.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/stores/diagnosticsStore.hpp"
#include "int/cWorldStateFunctions.hpp"
#include "cDiagnostics.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/stores/fieldDimensionsStore.hpp"
#include "int/stores/robotStore.hpp"


namespace hidden
{
    // TODO use proper data from configuration
    const double kMaxBallSpeed = 2.0;

    Point2D calculatePassDestination(const teamplay::robot &pass_receiving_robot)
    {
        const Point2D &own_location_raw = teamplay::robotStore::getInstance().getOwnRobot().getLocation();
        const Vector2D own_location(own_location_raw.x, own_location_raw.y);

        const Point2D &receiver_location_raw = pass_receiving_robot.getLocation();
        const Vector2D receiver_location(receiver_location_raw.x, receiver_location_raw.y);

        const Velocity2D &receiver_velocity_raw = pass_receiving_robot.getVelocity();
        const Vector2D receiver_velocity(receiver_velocity_raw.x, receiver_velocity_raw.y);

        boost::optional<Vector2D> pass_destination = geometry::KinematicIntersect::intersect(
            own_location, kMaxBallSpeed, receiver_location, receiver_velocity);

        if (!pass_destination.is_initialized())
        {
            return receiver_location_raw;
        }
        TRACE("Pass destination = {%lf, %lf}", pass_destination->x, pass_destination->y);
        return {pass_destination->x, pass_destination->y};
    }
} // namespace hidden

cActionPass::cActionPass()
{
    _actionParameters["passType"] = std::make_pair(defaultPassTypes, false);
    _intention.action = actionTypeEnum::PASS;
}

// Returns: PASSED    if the robot managed to pass
//          RUNNING   if the robot is busy trying to pass
//          FAILED    if something went wrong
behTreeReturnEnum cActionPass::execute(const std::map<std::string, std::string> &parameters)
{
    try
    {
        behTreeReturnEnum result = behTreeReturnEnum::RUNNING;

        const std::string parameter_key = "passType";
        const auto &parameter_it = parameters.find(parameter_key);
        if (parameter_it != parameters.end())
        {
            const std::string &parameter_value = parameter_it->second;
            const shootEnum pass_type = shootMapping[parameter_value];
            result = executePassType(pass_type);
        }
        else
        {
            TRACE("WARNING: cActionPass: no parameters given");
        }

        sendIntention();
        return result;
    }
    catch (std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        throw std::runtime_error(std::string("cActionPass::execute Linked to: ") + e.what());
    }
}

behTreeReturnEnum cActionPass::executePassType(const shootEnum pass_type)
{
    boost::optional<teamplay::robot> pass_receiving_robot;
    switch(pass_type)
    {
        case shootEnum::PASS_TOWARDS_NEAREST_TEAMMEMBER:
        {
            TRACE("INFO: cActionPass: PASS_TOWARDS_NEAREST_TEAMMEMBER");
            pass_receiving_robot = cWorldStateFunctions::getInstance().getClosestTeammember(false);
            break;
        }
        case shootEnum::PASS_TOWARDS_NEAREST_ATTACKER:
        {
            TRACE("INFO: cActionPass: PASS_TOWARDS_NEAREST_ATTACKER");
            pass_receiving_robot = cWorldStateFunctions::getInstance().getClosestAttacker();
            break;
        }
        case shootEnum::PASS_TOWARDS_FURTHEST_ATTACKER:
        {
            TRACE("INFO: cActionPass: PASS_TOWARDS_FURTHEST_ATTACKER");
            pass_receiving_robot = cWorldStateFunctions::getInstance().getClosestAttackerToOpponentGoal();
            break;
        }
        case shootEnum::PASS_TOWARDS_NEAREST_ATTACKER_ON_OPP_HALF:
        {
            TRACE("INFO: cActionPass: PASS_TOWARDS_NEAREST_ATTACKER_ON_OPP_HALF");
            pass_receiving_robot = cWorldStateFunctions::getInstance().getClosestAttacker(teamplay::fieldArea::OPP_SIDE);
            break;
        }
        case shootEnum::PASS_TOWARDS_FURTHEST_DEFENDER:
        {
            TRACE("INFO: cActionPass: PASS_TOWARDS_FURTHEST_DEFENDER");
            pass_receiving_robot = cWorldStateFunctions::getInstance().getClosestDefenderToOpponentGoal();
            break;
        }
        case shootEnum::PASS_TOWARDS_TIP_IN_POSITION:
        {
            TRACE("INFO: cActionPass: PASS_TOWARDS_TIP_IN_POSITION");

            Point2D tip_in_POI = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::TIP_IN);
            _intention.position.x = tip_in_POI.x;
            _intention.position.y = tip_in_POI.y;
            return pass(tip_in_POI.x, tip_in_POI.y);
        }
        default:
        {
            TRACE("Unknown pass type encountered");
            return behTreeReturnEnum::FAILED;
        }
    }
    if (!pass_receiving_robot)
    {
        TRACE("Unable to determine pass receiving robot");
        return behTreeReturnEnum::FAILED;
    }

    Point2D pass_destination;
    const bool use_active_intercept = teamplay::configurationStore::getConfiguration().isActiveInterceptEnabled();
    if (use_active_intercept)
    {
        pass_destination = hidden::calculatePassDestination(*pass_receiving_robot);
        TRACE("ActionPass: original={x=%lf, y=%lf}, new={x=%lf, y=%lf}",
            pass_receiving_robot->getLocation().x, pass_receiving_robot->getLocation().y,
            pass_destination.x, pass_destination.y);
    }
    else
    {
        pass_destination = pass_receiving_robot->getLocation();
        TRACE("ActionPass: {x=%lf, y=%lf}",
            pass_destination.x, pass_destination.y);
    }

    _intention.position.x = pass_destination.x;
    _intention.position.y = pass_destination.y;
    return pass(_intention.position.x, _intention.position.y);
}
