 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cWorldStateFunctions.cpp
 *
 *  Created on: Sep 18, 2015
 *      Author: Ivo Matthijssen
 */

#include "int/cWorldStateFunctions.hpp"

#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <math.h>
#include <iostream>
#include <limits>

#include <boost/algorithm/string.hpp>
#include "FalconsCommon.h"

#include "int/stores/ballStore.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/cWorldModelInterface.hpp"
#include "int/rules/ruleStimulatePassing.hpp"
#include "int/stores/fieldDimensionsStore.hpp"
#include "int/stores/gameStateStore.hpp"
#include "int/stores/obstacleStore.hpp"
#include "int/stores/robotStore.hpp"
#include "int/utilities/trace.hpp"
#include "int/utilities/timer.hpp"
#include "int/actions/cAbstractAction.hpp"

#include "int/adapters/cRTDBInputAdapter.hpp"

using std::exception;
using std::runtime_error;


static robotLocations getTeammembers(bool excludingOwnRobot = true)
{
    robotLocations retVal;
    auto robotList = teamplay::robotStore::getInstance().getAllRobotsExclOwnRobot();
    if (!excludingOwnRobot)
    {
        robotList = teamplay::robotStore::getInstance().getAllRobots();
    }
    for(auto it = robotList.begin(); it != robotList.end(); it++)
    {
        robotLocation robotLoc;
        robotNumber robotNr = it->getNumber();
        robotLoc.position = geometry::Pose2D(it->getPosition().x, it->getPosition().y, it->getPosition().phi);
        robotLoc.velocity = geometry::Velocity2D(it->getVelocity().x, it->getVelocity().y, it->getVelocity().phi);

        retVal.insert(std::pair<robotNumber, robotLocation>(robotNr, robotLoc));
    }
    return retVal;
}


cWorldStateFunctions::cWorldStateFunctions()
{
    _robotID = 0;

    _fieldWidth = teamplay::fieldDimensionsStore::getFieldDimensions().getWidth();
    _fieldLength = teamplay::fieldDimensionsStore::getFieldDimensions().getLength();

    _positionMargin = 0.15;
}

cWorldStateFunctions::~cWorldStateFunctions()
{

}

bool isShortTurnToGoalBlockedByOpponent(const std::map<std::string, std::string> &params)
{
    try
    {
        Position2D own_location = teamplay::robotStore::getInstance().getOwnRobot().getPosition();

        Point2D opp_goalline_center;
        opp_goalline_center = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OPP_GOALLINE_CENTER);

        // Get angle to goal, normalized to [-pi, pi]
        double target_angle = angle_between_two_points_0_2pi(own_location.x, own_location.y, opp_goalline_center.x, opp_goalline_center.y) - own_location.phi;
        target_angle = project_angle_mpi_pi(target_angle);
        TRACE("target angle robot to goal: ") << std::to_string(target_angle);

        auto opponents = teamplay::obstacleStore::getInstance().getAllObstacles();

        for (auto opponent = opponents.begin(); opponent != opponents.end(); opponent++)
        {
            auto opp_location = opponent->getLocation();
            if (calc_distance(own_location.x, own_location.y, opp_location.x, opp_location.y) < 1.5)
            {
                TRACE("opponent at (") << std::to_string(opp_location.x) << ", " << std::to_string(opp_location.y) << ") is within 1.5 meter";

                // The opponent is within 1.5 meter: get angle to opponent, normalized to [-pi, pi]
                double angle_to_opp = angle_between_two_points_0_2pi(own_location.x, own_location.y, opp_location.x, opp_location.y) - own_location.phi;
                angle_to_opp = project_angle_mpi_pi(angle_to_opp);
                TRACE("angle between robot and opponent: ") << std::to_string(angle_to_opp);

                if (target_angle <= 0.0 && angle_to_opp <= 0.0)
                {
                    if (fabs(angle_to_opp) < fabs(target_angle))
                    {
                        TRACE("the opponent blocks the short turn to goal (clockwise). returning true...");
                        return true;
                    }
                }
                if (target_angle >= 0.0 && angle_to_opp >= 0.0)
                {
                    if (fabs(angle_to_opp) < fabs(target_angle))
                    {
                        TRACE("the opponent blocks the short turn to goal (counterclockwise). returning true...");
                        return true;
                    }
                }
            }
        }

        return false;
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool isLongTurnToGoalBlockedByOpponent(const std::map<std::string, std::string> &params)
{
    try
    {
        Position2D own_location = teamplay::robotStore::getInstance().getOwnRobot().getPosition();

        Point2D opp_goalline_center;
        opp_goalline_center = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OPP_GOALLINE_CENTER);

        // Get angle to goal, normalized to [-pi, pi]
        double target_angle = angle_between_two_points_0_2pi(own_location.x, own_location.y, opp_goalline_center.x, opp_goalline_center.y) - own_location.phi;
        target_angle = project_angle_mpi_pi(target_angle);
        TRACE("target angle robot to goal: ") << std::to_string(target_angle);

        auto opponents = teamplay::obstacleStore::getInstance().getAllObstacles();

        for (auto opponent = opponents.begin(); opponent != opponents.end(); opponent++)
        {
            auto opp_location = opponent->getLocation();
            if (calc_distance(own_location.x, own_location.y, opp_location.x, opp_location.y) < 1.5)
            {
                TRACE("opponent at (") << std::to_string(opp_location.x) << ", " << std::to_string(opp_location.y) << ") is within 1 meter";

                // The opponent is within 1.5 meter: get angle to opponent, normalized to [-pi, pi]
                double angle_to_opp = angle_between_two_points_0_2pi(own_location.x, own_location.y, opp_location.x, opp_location.y) - own_location.phi;
                angle_to_opp = project_angle_mpi_pi(angle_to_opp);
                TRACE("angle between robot and opponent: ") << std::to_string(angle_to_opp);

                if (target_angle <= 0.0 && angle_to_opp <= 0.0)
                {
                    if (fabs(angle_to_opp) > fabs(target_angle))
                    {
                        TRACE("the opponent blocks the long turn to goal. returning true...");
                        return true;
                    }
                }
                if (target_angle <= 0.0 && angle_to_opp > 0.0)
                {
                    TRACE("the opponent blocks the long turn to goal. returning true...");
                    return true;
                }
                if (target_angle >= 0.0 && angle_to_opp >= 0.0)
                {
                    if (fabs(angle_to_opp) > fabs(target_angle))
                    {
                        TRACE("the opponent blocks the long turn to goal. returning true...");
                        return true;
                    }
                }
                if (target_angle >= 0.0 && angle_to_opp < 0.0)
                {
                    TRACE("the opponent blocks the long turn to goal. returning true...");
                    return true;
                }
            }
        }

        return false;
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool isValidNumberOfPassesGiven(const std::map<std::string, std::string> &params)
{
    try
    {
        return teamplay::ruleStimulatePassing::getInstance().isRuleValid();
    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool isBallInOwnPenaltyArea(const std::map<std::string, std::string> &params)
{
    try
    {
        return teamplay::ballStore::getBall().isInOwnPenaltyArea();
    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool isBallInOpponentPenaltyArea(const std::map<std::string, std::string> &params)
{
    try
    {
        return teamplay::ballStore::getBall().isInOpponentPenaltyArea();
    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool cWorldStateFunctions::isMemberInArea(areaName area, bool includeOwnRobot, bool includeGoalie)
{
    bool retVal = false;

    try
    {
        // get all team mates locations from WorldModelInterface
        robotLocations teamMembers = getTeammembers();

        // check for every team mate; iterate over vector since not fixed how many team mates are in play and are seen by vision
        robotLocations::iterator it = teamMembers.begin();
        if (!includeOwnRobot) {
            for(it = teamMembers.begin(); it != teamMembers.end(); it++)
            {
                // check current team member coordinates against the area match
                if (it->first == _robotID) {
                    teamMembers.erase(it);
                    break;
                }
            }
        }

        if (!includeGoalie) {
            for(it = teamMembers.begin(); it != teamMembers.end(); it++)
            {
                // check current team member coordinates against the area match
                if (getRobotRole(it->first) == treeEnum::R_GOALKEEPER) {
                    teamMembers.erase(it);
                    break;
                }
            }
        }

        for(it = teamMembers.begin(); ((it != teamMembers.end()) && (!retVal)); it++)
        {
            // check current team member coordinates against the area match
            retVal = cEnvironmentField::getInstance().isPositionInArea(
                    (float) it->second.position.getX(),
                    (float) it->second.position.getY(),
                    area,
                    _positionMargin);
        }

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
    return retVal;
}

bool doesTeamHaveBall(const std::map<std::string, std::string> &params)
{
    bool ret_val = false;

    try
    {
        auto all_robots = teamplay::robotStore::getInstance().getAllRobots();
        ret_val = std::any_of(all_robots.begin(), all_robots.end(),
                              [&](const teamplay::robot it){ return it.hasBall(); });
    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return ret_val;
}

/*! Calculate the distance from given robot to the ball
 *
 * @return distance.  (nan if no ball seen or own position unclear)
 *
 */
boost::optional<float> cWorldStateFunctions::ballDistance(const robotNumber &robotID)
{
    try
    {
        teamplay::ball ball = teamplay::ballStore::getBall();

        // check if there is a ball, it's possible that vision does not see the ball
        if (!ball.isLocationKnown())
        {
            return boost::none;  //if we don't see a ball then we cannot calculate the distance
        }
        Point3D ballPosition = ball.getPosition();

        // Get robot positions, don't forget own robot
        robotLocations teamMembers = getTeammembers(false);

        if(teamMembers.find(robotID) != teamMembers.end())
        {
            Position2D robotPos = getPosition2D(teamMembers.at(robotID).position);
            return float (calc_distance( ballPosition.x, ballPosition.y, robotPos.x, robotPos.y ));
        }
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
    return boost::none;
}

/*! get the x,y location of the closest team member to the specified location
 *
 * @param[in] location_x x of the reference location to compare with
 * @param[in] location_y y of the reference location to compare with
 */
void cWorldStateFunctions::getClosestMemberToLocationXY(double location_x, double location_y, bool includeOwnRobot, bool includeGoalie, bool &foundMember, uint8_t &robotID)
{   // returns robotID 0 and foundMember = 0 when no valid robot can be returned and inCludeOwnRobot is false
    foundMember = false;
    robotID = 0;

    try
    {
        // start with imaginary high value
        double shortestDistance=99.0;

        // First calculate my distance to ball (if included)
        if (includeOwnRobot)
        {
            Position2D robotPos = getLocationOfRobot(_robotID);

            double robotDistance = calc_distance(
                    location_x,
                    location_y,
                    robotPos.x,
                    robotPos.y
                );

            shortestDistance = robotDistance;
            foundMember = true;
            robotID = _robotID;
        }

        // get all team mates locations from WorldModelInterface
        robotLocations teamMembers = getTeammembers();

        // remove goalkeeper from the equation
        robotLocations::iterator it = teamMembers.begin();
        if (!includeGoalie) {
            for(it = teamMembers.begin(); it != teamMembers.end(); it++)
            {
                // Remove the goalie if he should not be included in the result
                if (getRobotRole(it->first) == treeEnum::R_GOALKEEPER) {
                    teamMembers.erase(it);
                    break;
                }
            }
        }

        // Find closest remaining member to the given location
        robotLocations::iterator robotIt = teamMembers.begin();
        for(robotIt = teamMembers.begin(); robotIt != teamMembers.end(); robotIt++)
        {
            double robotDistance = calc_distance(
                    location_x,
                    location_y,
                    robotIt->second.position.x,
                    robotIt->second.position.y
                );

            if(robotDistance < shortestDistance)
            {
                foundMember = true;
                robotID = robotIt->first;

                shortestDistance = robotDistance;
            }
        }
    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}


/*! get the x,y location of the closest team member of this robot
 *
 * @param[out] target_x  x location of the closest team member
 * @param[out] target_y  y location of the closest team member
 * @return true if the function was successful in its task
 *         false if the function was not successful in its task, do not trust target_x and target_y
 */

bool cWorldStateFunctions::getClosestTeammember(double &target_x, double &target_y, bool includeGoalie)
{   // returns false when no valid coordinates can be returned
    bool retVal = false;

    try
    {
        // Get team members
        // get all team mates locations from WorldModelInterface
        robotLocations teamMembers = getTeammembers();
        Position2D teamMemberLocation;
        Position2D closestLocation;
        Position2D myLocation =  getLocationOfRobot(_robotID);
        float shortestDistance = 100;

        // check for every team mate; iterate over vector since not fixed how many team mates are in play and are seen by vision
        robotLocations::iterator it = teamMembers.begin();
        if (!includeGoalie) {
            for(it = teamMembers.begin(); it != teamMembers.end(); it++)
            {
                // Remove the goalie if he should not be included in the result
                if (getRobotRole(it->first) == treeEnum::R_GOALKEEPER) {
                    teamMembers.erase(it);
                    break;
                }
            }
        }

        for(it = teamMembers.begin(); (it != teamMembers.end()) ; it++)
        {
            // check current team member coordinates against the area match
            teamMemberLocation = Position2D(it->second.position.getX(), it->second.position.getY(), it->second.position.getPhi());
            float distance = calc_distance(myLocation, teamMemberLocation);
            if (distance < shortestDistance)
            {
                shortestDistance = distance;
                closestLocation = teamMemberLocation;
                retVal = true;
            }
        }

        if(retVal)
        {
            target_x = closestLocation.x;
            target_y = closestLocation.y;
        }

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool cWorldStateFunctions::getClosestAttacker(areaName area, double &target_x, double &target_y)
{
    bool result = false;
    try
    {

        // get my own location
        Position2D myPos = getLocationOfRobot(_robotID);

        double distanceToClosestAttacker = 999.0;

        // Find attackers.
        boost::optional<robotNumber> attackerMain = getRobotWithRole(treeEnum::ATTACKER_MAIN);
        boost::optional<robotNumber> attackerAssist = getRobotWithRole(treeEnum::ATTACKER_ASSIST);

        robotLocations teamMembers = getTeammembers();

        // TODO: what if no attacker found in area??? this was already the case with 1 attacker-main and no attacker-assist

        if (attackerMain &&
        (*attackerMain != _robotID) &&
        (teamMembers.find(*attackerMain) != teamMembers.end()))// If attackerMain role exists, and it's not me
        {
            // Compute distance between attacker and me.
            Position2D robotPos = getPosition2D(teamMembers.at(*attackerMain).position);

            // check current team member coordinates against the area match
            if (cEnvironmentField::getInstance().isPositionInArea((float) robotPos.x, (float) robotPos.y, area,    _positionMargin))
            {
                double distance = calc_distance( myPos, robotPos );
                if (distance < distanceToClosestAttacker)
                {
                    distanceToClosestAttacker = distance;
                    target_x = robotPos.x;
                    target_y = robotPos.y;

                    result = true;
                }
            }
        }

        if (attackerAssist &&
        (*attackerAssist != _robotID) &&
        (teamMembers.find(*attackerAssist) != teamMembers.end())) // If attackerAssist role exists, and it's not me
        {
            // Compute distance between attacker and me.
            Position2D robotPos = getPosition2D(teamMembers.at(*attackerAssist).position);

            // check current team member coordinates against the area match
            if (cEnvironmentField::getInstance().isPositionInArea((float) robotPos.x, (float) robotPos.y, area,    _positionMargin))
            {
                double distance = calc_distance( myPos, robotPos );
                if (distance < distanceToClosestAttacker)
                {
                    distanceToClosestAttacker = distance;
                    target_x = robotPos.x;
                    target_y = robotPos.y;

                    result = true;
                }
            }
        }

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("cWorldStateFunctions::getClosestAttacker Linked to: ") + e.what());
    }

    return result;
}

void cWorldStateFunctions::getClosestAttackerToOpponentGoal(double &target_x, double &target_y)
{
    try
    {
        // goal opponent goal info
        Point2D oppGoallineCenter = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OPP_GOALLINE_CENTER);

        // Find attackers.
        boost::optional<robotNumber> attackerMain = getRobotWithRole(treeEnum::ATTACKER_MAIN);
        boost::optional<robotNumber> attackerAssist = getRobotWithRole(treeEnum::ATTACKER_ASSIST);

        robotLocations teamMembers = getTeammembers();

        double distanceToClosestAttacker = 999.0;

        if (attackerMain &&
        (*attackerMain != _robotID) &&
        (teamMembers.find(*attackerMain) != teamMembers.end()))// If attackerMain role exists, and it's not me
        {
            // Compute distance between attacker and me.
            Position2D robotPos = getPosition2D(teamMembers.at(*attackerMain).position);

            double distance = calc_distance(oppGoallineCenter.x, oppGoallineCenter.y , robotPos.x, robotPos.y);
            if (distance < distanceToClosestAttacker)
            {
                distanceToClosestAttacker = distance;
                target_x = robotPos.x;
                target_y = robotPos.y;
            }
        }

        if (attackerAssist &&
        (*attackerAssist != _robotID) &&
        (teamMembers.find(*attackerAssist) != teamMembers.end())) // If attackerAssist role exists, and it's not me
        {
            // Compute distance between attacker and me.
            Position2D robotPos = getPosition2D(teamMembers.at(*attackerAssist).position);

            double distance = calc_distance(oppGoallineCenter.x, oppGoallineCenter.y , robotPos.x, robotPos.y);
            if (distance < distanceToClosestAttacker)
            {
                distanceToClosestAttacker = distance;
                target_x = robotPos.x;
                target_y = robotPos.y;
            }
        }

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

}

void cWorldStateFunctions::getClosestDefender(double &target_x, double &target_y)
{
    try
    {
        // get my own location
        Position2D myPos = getLocationOfRobot(_robotID);

        double distanceToClosestDefender = 999.0;

        // Find defenders.
        boost::optional<robotNumber> defenderMain = getRobotWithRole(treeEnum::DEFENDER_MAIN);
        boost::optional<robotNumber> defenderAssist = getRobotWithRole(treeEnum::DEFENDER_ASSIST);

        robotLocations teamMembers = getTeammembers();

    if (defenderMain &&
       (*defenderMain != _robotID) &&
       (teamMembers.find(*defenderMain) != teamMembers.end()))// If defenderMain role exists, and it's not me
        {
            // Compute distance between defender and me.
            Position2D robotPos = getPosition2D(teamMembers.at(*defenderMain).position);

            double distance = calc_distance( myPos, robotPos );
            if (distance < distanceToClosestDefender)
            {
                distanceToClosestDefender = distance;
                target_x = robotPos.x;
                target_y = robotPos.y;
            }
        }

    if (defenderAssist &&
       (*defenderAssist != _robotID) &&
       (teamMembers.find(*defenderAssist) != teamMembers.end()))// If defenderAssist role exists, and it's not me
        {
            // Compute distance between defender and me.
            Position2D robotPos = getPosition2D(teamMembers.at(*defenderAssist).position);

            double distance = calc_distance( myPos, robotPos );
            if (distance < distanceToClosestDefender)
            {
                distanceToClosestDefender = distance;
                target_x = robotPos.x;
                target_y = robotPos.y;
            }
        }
    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

void cWorldStateFunctions::getClosestDefenderToOpponentGoal(double &target_x, double &target_y)
{
    try
    {
        // goal opponent goal info
        Point2D oppGoallineCenter = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OPP_GOALLINE_CENTER);

        // Find attackers.
        boost::optional<robotNumber> defenderMain = getRobotWithRole(treeEnum::DEFENDER_MAIN);
        boost::optional<robotNumber> defenderAssist = getRobotWithRole(treeEnum::DEFENDER_ASSIST);

        robotLocations teamMembers = getTeammembers();

        double distanceToClosestDefender = 999.0;

        if (defenderMain &&
        (*defenderMain != _robotID) &&
        (teamMembers.find(*defenderMain) != teamMembers.end()))// If defenderMain role exists, and it's not me
        {
            // Compute distance between attacker and me.
            Position2D robotPos = getPosition2D(teamMembers.at(*defenderMain).position);

            double distance = calc_distance(oppGoallineCenter.x, oppGoallineCenter.y , robotPos.x, robotPos.y);
            if (distance < distanceToClosestDefender)
            {
                distanceToClosestDefender = distance;
                target_x = robotPos.x;
                target_y = robotPos.y;
            }
        }

        if (defenderAssist &&
        (*defenderAssist != _robotID) &&
        (teamMembers.find(*defenderAssist) != teamMembers.end())) // If defenderAssist role exists, and it's not me
        {
            // Compute distance between attacker and me.
            Position2D robotPos = getPosition2D(teamMembers.at(*defenderAssist).position);

            double distance = calc_distance(oppGoallineCenter.x, oppGoallineCenter.y , robotPos.x, robotPos.y);
            if (distance < distanceToClosestDefender)
            {
                distanceToClosestDefender = distance;
                target_x = robotPos.x;
                target_y = robotPos.y;
            }
        }

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

}

/*! get the x,y location of the closest opponent to the specified location
 *
 * @param[out] target_x  x location of the closest opponent
 * @param[out] target_y  y location of the closest opponent
 * @param[in] location_x x of the reference location to compare with
 * @param[in] location_y y of the reference location to compare with
 * @return true if the function was successful in its task
 *         false if the function was not successful in its task, do not trust target_x and target_y
 */
bool cWorldStateFunctions::getClosestOpponentToLocationXY(float &target_x, float &target_y, float location_x, float location_y)
{   // returns false when no valid coordinates can be returned
    bool retVal = false;

    try
    {
        // get all opponent locations from WorldModelInterface
        auto opponents = teamplay::obstacleStore::getInstance().getAllObstacles();

        std::vector<robotLocation> robots;
        for(auto it = opponents.begin(); it != opponents.end(); it++)
        {
            robotLocation obstacleLocation;
            obstacleLocation.position.x = it->getLocation().x;
            obstacleLocation.position.y = it->getLocation().y;
            robots.push_back(obstacleLocation);
        }

        // store reference location
        Point2D my2DreferenceLocation;
        my2DreferenceLocation.x=(double) location_x;
        my2DreferenceLocation.y=(double) location_y;

        std::sort(robots.begin(), robots.end(), robotLocationSorter(my2DreferenceLocation));

        if(robots.size() > 0)
        {
            target_x = (float) robots.at(0).position.getX();
            target_y = (float) robots.at(0).position.getY();
            retVal = true;
        }
        else
        {
            retVal = false;
        }

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool cWorldStateFunctions::getSecondClosestOpponentToLocationXY(float &target_x, float &target_y, float location_x, float location_y)
{   // returns false when no valid coordinates can be returned
    bool retVal = false;

    try
    {
        // get all opponent locations from WorldModelInterface
        auto opponents = teamplay::obstacleStore::getInstance().getAllObstacles();

        std::vector<robotLocation> robots;
        for(auto it = opponents.begin(); it != opponents.end(); it++)
        {
            robotLocation obstacleLocation;
            obstacleLocation.position.x = it->getLocation().x;
            obstacleLocation.position.y = it->getLocation().y;
            robots.push_back(obstacleLocation);
        }

        // store reference location
        Point2D my2DreferenceLocation;
        my2DreferenceLocation.x=(double) location_x;
        my2DreferenceLocation.y=(double) location_y;

        std::sort(robots.begin(), robots.end(), robotLocationSorter(my2DreferenceLocation));

        if(robots.size() > 1)
        {
            target_x = (float) robots.at(1).position.getX();
            target_y = (float) robots.at(1).position.getY();
            retVal = true;
        }
        else
        {
            retVal = false;
        }

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

/*! get the x,y location of the closest opponent to this robot
 *
 * @param[out] target_x  x location of the opponent
 * @param[out] target_y  y location of the opponent
 * @return true if the function was successful in its task
 *         false if the function was not successful in its task, do not trust target_x and target_y
 */
bool cWorldStateFunctions::getClosestOpponent(float &target_x, float &target_y)
{   // returns false when no valid coordinates can be returned
    bool retVal = false;

    try
    {
        // get my own location
        Position2D my2DPosition = getLocationOfRobot(_robotID);

        retVal = cWorldStateFunctions::getClosestOpponentToLocationXY( target_x, target_y, (float) my2DPosition.x, (float) my2DPosition.y );

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool cWorldStateFunctions::getPotentialOpponentAttacker(float &target_x, float &target_y)
{   // returns false when no valid coordinates can be returned
    bool retVal = false;

    try
    {
        // Get closest opponent to own goal
        Point2D own_goalline_center;
        own_goalline_center = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OWN_GOALLINE_CENTER);
        float closestOpponentToOwnGoal_x;
        float closestOpponentToOwnGoal_y;
        bool closestOpponentToOwnGoalExists = cWorldStateFunctions::getClosestOpponentToLocationXY(closestOpponentToOwnGoal_x, closestOpponentToOwnGoal_y,
                (float) own_goalline_center.x, (float) own_goalline_center.y);

        // Get closest opponent to ball position
        teamplay::ball ball = teamplay::ballStore::getBall();
        float closestOpponentToBall_x;
        float closestOpponentToBall_y;
        bool closestOpponentToBallExists = cWorldStateFunctions::getClosestOpponentToLocationXY(closestOpponentToBall_x, closestOpponentToBall_y,
                (float) ball.getPosition().x, (float) ball.getPosition().y);

        // Check if opponents have been found
        if (closestOpponentToOwnGoalExists && closestOpponentToBallExists)
        {
            // Check if closest opponent to goal is the same as closest to ball, then take second closest opponent
            if (calc_distance(closestOpponentToOwnGoal_x, closestOpponentToOwnGoal_y, closestOpponentToBall_x, closestOpponentToBall_y) > 0.2)
            {
                target_x = closestOpponentToOwnGoal_x;
                target_y = closestOpponentToOwnGoal_y;
                retVal = true;
            }
            else
            {
                // get second closest opponent to own goal (only if available)
                if (cWorldStateFunctions::getSecondClosestOpponentToLocationXY(closestOpponentToOwnGoal_x, closestOpponentToOwnGoal_y,
                        (float) own_goalline_center.x, (float) own_goalline_center.y))
                {
                    target_x = closestOpponentToOwnGoal_x;
                    target_y = closestOpponentToOwnGoal_y;
                    retVal = true;
                }
            }
        }

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

boost::optional<robotNumber> cWorldStateFunctions::getRobotWithRole(const treeEnum &role)
{
    try
    {
        auto robot_with_role = teamplay::robotStore::getInstance().getRobotWithRole(role);
        if (robot_with_role)
        {
            return robot_with_role->getNumber();
        }
        else
        {
            return boost::none;
        }
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}


bool isLowestActiveRobotID(const std::map<std::string, std::string> &params)
{
    try
    {
        auto own_robot_id = teamplay::robotStore::getInstance().getOwnRobot().getNumber();
        auto all_robots = teamplay::robotStore::getInstance().getAllRobots();

        auto robots_with_lower_id = std::count_if(all_robots.begin(), all_robots.end(),
                   [&](const teamplay::robot& it){ return it.getNumber() < own_robot_id; });

        return robots_with_lower_id == 0;
    }
    catch (exception &e)
    {
        TRACE_ERROR(e.what());
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool isHighestActiveRobotID(const std::map<std::string, std::string> &params)
{
    try
    {
        auto own_robot_id = teamplay::robotStore::getInstance().getOwnRobot().getNumber();
        auto all_robots = teamplay::robotStore::getInstance().getAllRobots();

        auto robots_with_higher_id = std::count_if(all_robots.begin(), all_robots.end(),
                   [&](const teamplay::robot& it){ return it.getNumber() > own_robot_id; });

        return robots_with_higher_id == 0;
    }
    catch (exception &e)
    {
        TRACE_ERROR(e.what());
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool isSecondHighestActiveRobotID(const std::map<std::string, std::string> &params)
{
    try
    {
        auto own_robot_id = teamplay::robotStore::getInstance().getOwnRobot().getNumber();
        auto all_robots = teamplay::robotStore::getInstance().getAllRobots();

        auto robots_with_higher_id = std::count_if(all_robots.begin(), all_robots.end(),
                   [&](const teamplay::robot& it){ return it.getNumber() > own_robot_id; });

        return robots_with_higher_id == 1;
    }
    catch (exception &e)
    {
        TRACE_ERROR(e.what());
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool isThirdHighestActiveRobotID(const std::map<std::string, std::string> &params)
{
    try
    {
        auto own_robot_id = teamplay::robotStore::getInstance().getOwnRobot().getNumber();
        auto all_robots = teamplay::robotStore::getInstance().getAllRobots();

        auto robots_with_higher_id = std::count_if(all_robots.begin(), all_robots.end(),
                   [&](const teamplay::robot& it){ return it.getNumber() > own_robot_id; });

        return robots_with_higher_id == 2;
    }
    catch (exception &e)
    {
        TRACE_ERROR(e.what());
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool isOnlyActiveRobotID(const std::map<std::string, std::string> &params)
{
    try
    {
        auto all_robots = teamplay::robotStore::getInstance().getAllRobots();

        return all_robots.size() == 1;
    }
    catch (exception &e)
    {
        TRACE_ERROR(e.what());
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool isInMatch(const std::map<std::string, std::string> &params)
{
    return teamplay::gameStateStore::getInstance().getGameState().isInMatch();
}

bool isSetPiece(const std::map<std::string, std::string> &params)
{
    return teamplay::gameStateStore::getInstance().getGameState().isSetPiece();
}

bool isOwnSetPiece(const std::map<std::string, std::string> &params)
{
    return teamplay::gameStateStore::getInstance().getGameState().isOwnSetPiece();
}

bool isPrepareSetPiece(const std::map<std::string, std::string> &params)
{
    return teamplay::gameStateStore::getInstance().getGameState().isPrepareSetPiece();
}

bool isKickoffSetPiece(const std::map<std::string, std::string> &params)
{
    return teamplay::gameStateStore::getInstance().getGameState().isKickoffSetPiece();
}

bool isDroppedBallSetPiece(const std::map<std::string, std::string> &params)
{
    return teamplay::gameStateStore::getInstance().getGameState().isDroppedBallSetPiece();
}

bool isPenaltySetPiece(const std::map<std::string, std::string> &params)
{
    return teamplay::gameStateStore::getInstance().getGameState().isPenaltySetPiece();
}

bool isGoalkickSetPiece(const std::map<std::string, std::string> &params)
{
    return teamplay::gameStateStore::getInstance().getGameState().isGoalkickSetPiece();
}

bool isFreekickSetPiece(const std::map<std::string, std::string> &params)
{
    return teamplay::gameStateStore::getInstance().getGameState().isFreekickSetPiece();
}

bool isCornerSetPiece(const std::map<std::string, std::string> &params)
{
    return teamplay::gameStateStore::getInstance().getGameState().isCornerSetPiece();
}

bool isThrowinSetPiece(const std::map<std::string, std::string> &params)
{
    return teamplay::gameStateStore::getInstance().getGameState().isThrowinSetPiece();
}


robotNumber cWorldStateFunctions::getRobotID()
{
    return _robotID;
}

void cWorldStateFunctions::setRobotID(const robotNumber &robotID)
{
    _robotID = robotID;
}

void cWorldStateFunctions::setRobotRole(const robotNumber &robotID, const treeEnum &role)
{
    _robotRoles[robotID] = role;
}

treeEnum cWorldStateFunctions::getRobotRole(const robotNumber &robotID)
{
    treeEnum result = treeEnum::INVALID;
    try
    {
        if (_robotRoles.find(robotID) != _robotRoles.end())
        {
            result = _robotRoles.at(robotID);
        }
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
    return result;
}


bool within1mOfBall(const std::map<std::string, std::string> &params)
{
    bool retVal = false;

    try
    {
        Position2D myPos = cWorldStateFunctions::getInstance().getLocationOfRobot(cWorldStateFunctions::getInstance().getRobotID());
        teamplay::ball ball = teamplay::ballStore::getBall();

        if (ball.isLocationKnown())
        {
            Point3D ball_location = ball.getPosition();
            Vector2D delta = Vector2D(ball_location.x, ball_location.y) - myPos.xy();
            if (delta.size() < 1.0)
            {
                retVal = true;
            }
        }
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool doesOwnRobotHaveBall(const std::map<std::string, std::string> &params)
{
    bool retVal = false;

    try
    {
        retVal = teamplay::robotStore::getInstance().getOwnRobot().hasBall();
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool isOwnRobotAtOpponentSide(const std::map<std::string, std::string> &params)
{
    try
    {
        Position2D ownPos = teamplay::robotStore::getInstance().getOwnRobot().getPosition();
        return teamplay::fieldDimensionsStore::getFieldDimensions().isPositionInOpponentSide(ownPos.x, ownPos.y);

    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool isBallAtSide(const std::map<std::string, std::string> &params)
{
    bool retVal = false; // default no valid location given or no ball found in areas

    try
    {
        // read out side parameter: ball_own_opp = OWN or OPP. ball_left_right = LEFT or RIGHT
        // The side or quadrant at which to validate ball position
        std::string sideStrOwnOpp("ball_own_opp");
        auto paramValPairOwnOpp = params.find(sideStrOwnOpp);
        std::string sideStrLeftRight("ball_left_right");
        auto paramValPairLeftRight = params.find(sideStrLeftRight);

        if(paramValPairOwnOpp != params.end())
        {
            std::string paramValOwnOpp = paramValPairOwnOpp->second;
            if(paramValPairLeftRight != params.end())
            {
                std::string paramValLeftRight = paramValPairLeftRight->second;

                // evaluate input values to check ball location in quadrants
                if((paramValOwnOpp == "OWN") && (paramValLeftRight == "LEFT"))
                {
                    retVal = ((teamplay::ballStore::getBall().isAtOwnSide()) && (teamplay::ballStore::getBall().isAtLeftSide()));
                }
                else if((paramValOwnOpp == "OWN") && (paramValLeftRight == "RIGHT"))
                {
                    retVal = ((teamplay::ballStore::getBall().isAtOwnSide()) && (teamplay::ballStore::getBall().isAtRightSide()));
                }
                else if((paramValOwnOpp == "OPP") && (paramValLeftRight == "LEFT"))
                {
                    retVal = ((teamplay::ballStore::getBall().isAtOpponentSide()) && (teamplay::ballStore::getBall().isAtLeftSide()));
                }
                else if((paramValOwnOpp == "OPP") && (paramValLeftRight == "RIGHT"))
                {
                    retVal = ((teamplay::ballStore::getBall().isAtOpponentSide()) && (teamplay::ballStore::getBall().isAtRightSide()));
                }

                // if only one parameter is filled, check ball location in side of field (own,opp,left or right)
                else if ((paramValOwnOpp == emptyValue) || (paramValLeftRight == emptyValue))
                {
                    if (paramValOwnOpp == "OWN")
                    {
                        retVal = teamplay::ballStore::getBall().isAtOwnSide();
                    }
                    else if (paramValOwnOpp == "OPP")
                    {
                        retVal = teamplay::ballStore::getBall().isAtOpponentSide();
                    }
                    else if (paramValLeftRight == "LEFT")
                    {
                        retVal = teamplay::ballStore::getBall().isAtLeftSide();
                    }
                    else if (paramValLeftRight == "RIGHT")
                    {
                        retVal = teamplay::ballStore::getBall().isAtRightSide();
                    }
                }
            }
        }
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool isOwnRobotClosestToPOI(const std::map<std::string, std::string> &params)
{
    bool retVal = false; // default no POI given

    try
    {
        // Retrieve POI from parameters
        std::string poiStr("POI");
        boost::optional<Position2D> POI = cWorldStateFunctions::getInstance().getPos2DFromStr(params, poiStr);

        if (POI)
        {
            auto sortedListOfRobots = teamplay::robotStore::getInstance().getAllRobotsExclLowestIDSortedByDistanceTo(Point2D(POI->x,POI->y));
            // 0th element in row always exists and is closest to POI
            retVal = sortedListOfRobots.at(0).isOwnRobot();
        }
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool isOwnRobotSecondClosestToPOI(const std::map<std::string, std::string> &params)
{
    bool retVal = false; // default no POI given

    try
    {
        // Retrieve POI from parameters
        std::string poiStr("POI");
        boost::optional<Position2D> POI = cWorldStateFunctions::getInstance().getPos2DFromStr(params, poiStr);

        if (POI)
        {
            auto sortedListOfRobots = teamplay::robotStore::getInstance().getAllRobotsExclLowestIDSortedByDistanceTo(Point2D(POI->x,POI->y));
            // check if 2 or more robots are active, 1th element is second closest to POI
            if (sortedListOfRobots.size() > 1)
            {
                retVal = sortedListOfRobots.at(1).isOwnRobot();
            }
        }
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool isOwnRobotFurthestFromPOI(const std::map<std::string, std::string> &params)
{
    bool retVal = false; // default no POI given

    try
    {
        // Retrieve POI from parameters
        std::string poiStr("POI");
        boost::optional<Position2D> POI = cWorldStateFunctions::getInstance().getPos2DFromStr(params, poiStr);

        if (POI)
        {
            auto sortedListOfRobots = teamplay::robotStore::getInstance().getAllRobotsExclLowestIDSortedByDistanceTo(Point2D(POI->x,POI->y));
            if (sortedListOfRobots.size() > 0)
            {
                retVal = sortedListOfRobots.at(sortedListOfRobots.size()-1).isOwnRobot();
            }
        }
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

// hack WSF only for WC Canada 2018!
bool isOwnRobotSetpieceDefenderAssist(const std::map<std::string, std::string> &params)
{
    bool retVal = false; // default no POI given

    try
    {
        // Retrieve POI from parameters
        std::string poiStr("POI");
        boost::optional<Position2D> POI = cWorldStateFunctions::getInstance().getPos2DFromStr(params, poiStr);
        Point2D ownGoallineCenter = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OWN_GOALLINE_CENTER);

        if (POI)
        {
            auto sortedListOfRobots = teamplay::robotStore::getInstance().getAllRobotsExclLowestIDSortedByDistanceTo(Point2D(POI->x,POI->y));

            if (sortedListOfRobots.size() == 3)
            {
                retVal = sortedListOfRobots.at(2).isOwnRobot();
            }

            if (sortedListOfRobots.size() == 4)
            {
                if (sortedListOfRobots.at(2).isOwnRobot())
                {
                    retVal = sortedListOfRobots.at(2).getDistanceTo(ownGoallineCenter) < sortedListOfRobots.at(3).getDistanceTo(ownGoallineCenter);
                }
                if (sortedListOfRobots.at(3).isOwnRobot())
                {
                    retVal = sortedListOfRobots.at(3).getDistanceTo(ownGoallineCenter) < sortedListOfRobots.at(2).getDistanceTo(ownGoallineCenter);
                }
            }
        }
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool isOwnRobotNearestToBall(const std::map<std::string, std::string> &params)
{
    bool retVal = false;

    try
    {
        std::pair<teamplay::robot, double> quickest_to_ball;
        quickest_to_ball.second = std::numeric_limits<double>::max();

        auto robots = teamplay::robotStore::getInstance().getAllRobotsExclGoalie();
        for (auto robot = robots.begin(); robot != robots.end(); ++robot)
        {
            auto time_to_ball = cRTDBInputAdapter::getInstance().getMPClient().getTimeToBall(robot->getNumber());
            if (time_to_ball < quickest_to_ball.second)
            {
                quickest_to_ball.first = *robot;
                quickest_to_ball.second = time_to_ball;
            }
        }

        retVal = quickest_to_ball.first.isOwnRobot();
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool isOwnRobotNearestToLastKnownBallLocation(const std::map<std::string, std::string> &params)
{
    bool retVal = false;

    try
    {
        uint8_t robotID = 0;
        bool memberFound = false;
        teamplay::ball ball = teamplay::ballStore::getBall();

        Point3D ball_location = ball.getPosition();

        cWorldStateFunctions::getInstance().getClosestMemberToLocationXY(
                ball_location.x,
                ball_location.y,
                true, // Include own robot
                false, // Exclude goalie
                memberFound,
                robotID
            );

        retVal = ((robotID == cWorldStateFunctions::getInstance().getRobotID()) && (memberFound));
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool isBallLocationKnown(const std::map<std::string, std::string> &params)
{
    return teamplay::ballStore::getBall().isLocationKnown();
}

/*!
 * \brief Is my assistant present?
 *
 * ATTACKER_MAIN checks for ATTACKER_ASSIST and vice versa
 * DEFENDER_MAIN checks for DEFENDER_ASSIST and vice versa
 * All others return false.
 */
bool isAssistentPresent(const std::map<std::string, std::string> &params)
{
    try
    {
        return teamplay::robotStore::getInstance().getAssistantOfOwnRobot() != boost::none;
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool isPotentialOppAttackerPresent(const std::map<std::string, std::string> &params)
{
    bool retVal = false;

    try
    {
        float target_x;
        float target_y;
        retVal = cWorldStateFunctions::getInstance().getPotentialOpponentAttacker(target_x, target_y);
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

teamplay::robot getRobotClosestToPoint(const teamplay::robots& robots, const Point2D& point)
{
    if (robots.size() < 1)
    {
        throw std::runtime_error("getRobotClosestToPoint cannot handle an empty list");
    }

    teamplay::robot closest_robot = robots.at(0);

    try
    {
        double closest_distance = calc_distance(closest_robot.getLocation(), point);

        teamplay::robots::const_iterator it;
        for (it = robots.begin(); it != robots.end(); ++it)
        {
            double distance = calc_distance(it->getLocation(), point);
            if (distance < closest_distance)
            {
                closest_robot = *it;
                closest_distance = distance;
            }
        }
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
    return closest_robot;
}

Position2D cWorldStateFunctions::getLocationOfRobot(const robotNumber &robotID)
{
    // If the given robotNr is my own robot nr, return my own location
    if (robotID == cWorldStateFunctions::getInstance().getRobotID())
    {
        Position2D ownPos = teamplay::robotStore::getInstance().getOwnRobot().getPosition();

        return ownPos;
    }
    else
    {
        // Otherwise, return the given robot's location
        robotLocations teamMembers = getTeammembers();

        robotLocations::iterator it;
        for (it = teamMembers.begin(); it != teamMembers.end(); ++it)
        {
            if (it->first == robotID)
            {
                Position2D pos = Position2D(it->second.position.getX(), it->second.position.getY(), it->second.position.getPhi());
                return pos;
            }
        }
    }
    return Position2D(); // Member not found. Return empty position.
}

bool isOpponentGoalKeeperInLeftCorner(const std::map<std::string, std::string> &params)
{
    try
    {
        // Select closest opponent to opponent goal (= opponent goalkeeper)
        Point2D oppGoallineCenter = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OPP_GOALLINE_CENTER);

        float oppGoalKeeper_x;
        float oppGoalKeeper_y;
        cWorldStateFunctions::getInstance().getClosestOpponentToLocationXY(oppGoalKeeper_x, oppGoalKeeper_y, (float) oppGoallineCenter.x, (float) oppGoallineCenter.y );

        Point2D distThreshold;
        distThreshold.x = 0.25; distThreshold.y = 1.5;

        // Check if closest opponent is a goalkeeper (y >= 7.5) and positioned in left corner (x < 0.0)
        if ((oppGoalKeeper_x < (oppGoallineCenter.x - distThreshold.x))
                && (oppGoalKeeper_y > (oppGoallineCenter.y - distThreshold.y)))
        {
            return true;
        }
        else
        {
            return false;
        }

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool isOpponentGoalKeeperInRightCorner(const std::map<std::string, std::string> &params)
{
    try
    {
        // Select closest opponent to opponent goal (= opponent goalkeeper)
        Point2D oppGoallineCenter = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OPP_GOALLINE_CENTER);

        float oppGoalKeeper_x;
        float oppGoalKeeper_y;
        cWorldStateFunctions::getInstance().getClosestOpponentToLocationXY(oppGoalKeeper_x, oppGoalKeeper_y, (float) oppGoallineCenter.x, (float) oppGoallineCenter.y  );

        Point2D distThreshold;
        distThreshold.x = 0.25; distThreshold.y = 1.5;

        // Check if closest opponent is a goalkeeper (y >= 7.5) and positioned in right corner (x > 0.0)
        if ((oppGoalKeeper_x > (oppGoallineCenter.x + distThreshold.x))
                && (oppGoalKeeper_y > (oppGoallineCenter.y - distThreshold.y)))
        {
            return true;
        }
        else
        {
            return false;
        }

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool isBallApproachingRobot(const std::map<std::string, std::string> &params)
{
    try
    {
        auto captureRadius = teamplay::configurationStore::getConfiguration().getInterceptBallCaptureRadius();
        if (params.count("captureRadius") && params.at("captureRadius").compare(emptyValue) != 0)
        {
            captureRadius = boost::lexical_cast<float>(params.at("captureRadius"));
        }
        auto minimumSpeed = teamplay::configurationStore::getConfiguration().getInterceptBallMinimumSpeed();

        // get robot position
        Position2D currentPos = teamplay::robotStore::getInstance().getOwnRobot().getPosition();
        geometry::Pose2D currentPose(currentPos.x, currentPos.y, currentPos.phi);

        // get ball data and position
        teamplay::ball ball = teamplay::ballStore::getBall();
        geometry::Pose2D ballPosition(ball.getPosition().x, ball.getPosition().y, 0);
        geometry::Velocity2D ballVelocity(ball.getVelocity().x, ball.getVelocity().y, 0);

        // determine relative speed
        geometry::Pose2D ballPositionTransformed = ballPosition;
        geometry::Pose2D ballPositionRCS = ballPositionTransformed.transformFCS2RCS(currentPose);
        geometry::Velocity2D ballVelocityTransformed = ballVelocity;
        geometry::Velocity2D ballVelocityRCS = ballVelocityTransformed.transformFCS2RCS(currentPose);
        float ballSpeed = ballVelocity.size();
        Vector2D ballSpeedVec2D(ball.getVelocity().x, ball.getVelocity().y);
        Vector2D ballPositionVec2D(ball.getPosition().x, ball.getPosition().y);

        bool ballIsMovingTowardsUs = (ballVelocityRCS.y < 0);
        bool ballMovingFastEnough = (ballSpeed > minimumSpeed);

        //// Compute flag: ballIntersectsCaptureRadius
        bool ballIntersectsCaptureRadius = false;
        // span a strafing line using RCS coordinates
        Position2D leftPosRcs(-captureRadius, 0, 0);
        Position2D rightPosRcs(captureRadius, 0, 0);

        // modify currentpos, as if already facing the ball, so we can use transformation
        Position2D transformedCurrentPos = currentPos;
        transformedCurrentPos.phi = angle_between_two_points_0_2pi(transformedCurrentPos.x, transformedCurrentPos.y, ball.getPosition().x, ball.getPosition().y);
        Position2D leftPosFcs = leftPosRcs.transform_rcs2fcs(transformedCurrentPos);
        Position2D rightPosFcs = rightPosRcs.transform_rcs2fcs(transformedCurrentPos);

        // intersect
        Vector2D ballProjection = ballPositionVec2D + (10.0 * ballSpeedVec2D); // make vector long enough
        Vector2D intersectResult;
        Position2D intersectPos;
        if (intersect(ballPosition, ballProjection, leftPosFcs.xy(), rightPosFcs.xy(), intersectResult))
        {
            if ((intersectResult - currentPos.xy()).size() < captureRadius)
            {
                //TRACE("intercept: ") << std::to_string(intersectResult.x) << ", " << std::to_string(intersectResult.y);
                intersectPos.phi = transformedCurrentPos.phi;
                intersectPos.x = intersectResult.x;
                intersectPos.y = intersectResult.y;
                ballIntersectsCaptureRadius = true;
            }
        }

        //TRACE("captureRadius=") << std::to_string(captureRadius)
        //    << " ballIntersectsCaptureRadius=" << std::to_string(ballIntersectsCaptureRadius)
        //    << " ballIsMovingTowardsUs=" << std::to_string(ballIsMovingTowardsUs)
        //    << " ballMovingFastEnough=" << std::to_string(ballMovingFastEnough);
        return (ballIntersectsCaptureRadius && ballIsMovingTowardsUs && ballMovingFastEnough);

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool isPassApproachingRobot(const std::map<std::string, std::string> &params)
{
    bool retVal = false;
    try 
    {
        // same as isBallApproachingRobot, but with extra conditions:
        // - pass intention must be given
        // - pass shot must have been performed
        // - timer must not have elapsed

        // Remember the location where the pass is going towards
        bool closestRobotToPassTarget = false;

        // check the state, update timer
        static teamplay::timer lastPassIntention;
        auto robotList = teamplay::robotStore::getInstance().getAllRobotsExclOwnRobot();
        for(auto it = robotList.begin(); it != robotList.end(); it++)
        {
            auto intention = cRTDBInputAdapter::getInstance().getIntention(it->getNumber());
            if (intention.action == actionTypeEnum::PASS)
            {
                lastPassIntention.reset();

                // Also check if this robot is the closest robot to the pass target
                std::vector<teamplay::robot> robotsByDistance = teamplay::robotStore::getInstance().getAllRobotsExclGoalieSortedByDistanceTo( Point2D(intention.position.x, intention.position.y) );
                if (robotsByDistance.at(0).isOwnRobot())
                {
                    closestRobotToPassTarget = true;
                }
            }
        }
        
        // check if pass has been given recently enough
        auto distance = cWorldStateFunctions::getInstance().ballDistance(teamplay::robotStore::getInstance().getOwnRobot().getNumber());
        double timeout = 10.0;
        if (distance)
        {
            float speed = 1.0; // TODO make speed scaling configurable
            timeout = *distance / speed;
        }
        teamplay::ball ball = teamplay::ballStore::getBall();
        bool ballLost = !ball.isLocationKnown();
        bool teammateRecentPassIntention = !lastPassIntention.hasElapsed(timeout);
        if (teammateRecentPassIntention)
        {
            // timers not yet expired -> check ball movement
            // note: intention also includes (x,y) coordinates, but there is no need to use these
            auto paramsOverride = params;
            if (!params.count("captureRadius"))
            {
                paramsOverride["captureRadius"] = "3.0"; // default 1.0m is far too little
            }

            // TODO: also override minimum ball speed to a lower value? (ball pass may be very weak)
            bool ballApproaching = isBallApproachingRobot(paramsOverride);
            if (ballLost || ballApproaching || closestRobotToPassTarget)
            {
                retVal = true;
                TRACE("staying in place to intercept");
            }
        }
    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
    return retVal;
}

bool isOpponentHalfReachable(const std::map<std::string, std::string> &params)
{
    bool retVal = false; // initially opponent half is not reachable

    try
    {
        // get ball data
        teamplay::ball ball = teamplay::ballStore::getBall();

        double line_y_threshold = -2.0; // distance for which we can reach opponent half with ball (depending on current driving speed)

        if (ball.getClaimedPosition().y > line_y_threshold)
        {
            retVal = true;
        }

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

void cWorldStateFunctions::getObstructingObstaclesInPath(const Point2D robotPos, const Point2D targetPos, const float radiusObjectMeters, std::vector<robotLocation> &obstacles)
{
    /*
     * Steps:
     * 1) Clear vector for consistent output
     * 2) Filter out friendly objects that are in reach of target position
     * 3) Add obstacle objects in set
     * 4) Calculate group B of pathplanning XY algorithm for obstacle avoidance
     */
    try
    {
        std::vector<robotLocation> inputSet;

        Position2D targetPos2D = Position2D(targetPos.x, targetPos.y, 0.0);
        Position2D robotPos2D = Position2D(robotPos.x, robotPos.y, 0.0);

        // Step 1
        obstacles.clear();

        // Step 2
        robotLocations teammembers = getTeammembers();

        for(robotLocations::iterator i = teammembers.begin(); i != teammembers.end(); i++)
        {
            robotLocation teammember = i->second;
            Position2D teammember2D = Position2D(teammember.position.getX(), teammember.position.getY(), 0.0);

            // If radius is not smaller than add
            if(!( ((teammember2D - targetPos2D).size() < radiusObjectMeters) ||
                  ((teammember2D - robotPos2D).size() < radiusObjectMeters) )
              )
            {
                inputSet.push_back(teammember);
            }
        }

        // Step 3 Add all obstacles
        auto opponents = teamplay::obstacleStore::getInstance().getAllObstacles();

        for(auto i = opponents.begin(); i != opponents.end(); i++)
        {
            robotLocation obstacleLocation;
            obstacleLocation.position.x = i->getLocation().x;
            obstacleLocation.position.y = i->getLocation().y;
            inputSet.push_back(obstacleLocation);
        }

        // Step 4 Calculate set B
        // 1. Determine B = Set of all obstacles in the direct path from robot position to target position, taking also the diameter of the obstacles and the robot into account.
        for(size_t i = 0; i < inputSet.size(); i++)
        {
            double a_i;
            double b_i;

            // Pre-compute a_i and b_i for all obstacles, and store in the struct for later reuse.
            Vector2D obstVec = Vector2D(inputSet.at(i).position.getX(), inputSet.at(i).position.getY());
            a_i = ((targetPos2D.xy() - robotPos2D.xy()) * (obstVec - robotPos2D.xy())) / (targetPos2D.xy() - robotPos2D.xy()).size();
            b_i = ((targetPos2D.xy() - robotPos2D.xy()).CrossProduct((obstVec - robotPos2D.xy()))) / (targetPos2D.xy() - robotPos2D.xy()).size();

            // Equation (4) from the paper.
            if ( ( 0 < a_i ) &&
                 ( a_i <= (targetPos2D.xy() - robotPos2D.xy()).size() ) &&
                 ( fabs(b_i) < (ROBOT_RADIUS + radiusObjectMeters) )) // r_r + r_i : robot radius + obstacle radius
            {
                obstacles.push_back(inputSet.at(i));
            }

        }

        // Sort the obstacle on nearest to robotPos
        std::sort(obstacles.begin(), obstacles.end(), robotLocationSorter(robotPos));

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

}

bool isShotOnGoalBlocked(const std::map<std::string, std::string> &params)
{
    bool retVal = false; // initially shot on goal is not blocked

    try
    {
        double shootPathRadius = teamplay::configurationStore::getConfiguration().getShootPathWidth() / 2.0;
        double opp_goalkeeper_positionmargin = 0.5;

        Position2D robotPos2D = teamplay::robotStore::getInstance().getOwnRobot().getPosition();
        Point2D robotPos(robotPos2D.x, robotPos2D.y);

        float preferred_shoottarget_x;
        float preferred_shoottarget_y;
        cWorldStateFunctions::getInstance().getPreferredShootXYOfGoal(preferred_shoottarget_x, preferred_shoottarget_y);
        Point2D preferredPartOfGoal;
        preferredPartOfGoal.x = preferred_shoottarget_x;
        preferredPartOfGoal.y = preferred_shoottarget_y;

        std::vector<robotLocation> obstacles;

        cWorldStateFunctions::getInstance().getObstructingObstaclesInPath(robotPos, preferredPartOfGoal, shootPathRadius, obstacles);

        for(size_t i = 0; i < obstacles.size(); i++)
        {
            // Check if closest opponent is not in opponent goalarea (= opponent goalkeeper)
            if (!cEnvironmentField::getInstance().isPositionInArea(
                    obstacles.at(i).position.getX(), obstacles.at(i).position.getY(),
                    A_OPP_GOALAREA, opp_goalkeeper_positionmargin))
            {
                retVal = true; //shot on goal is blocked by an opponent
            }
        }

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool isLobShotOnGoalBlocked(const std::map<std::string, std::string> &params)
{
    bool retVal = false; // initially shot on goal is not blocked

    try
    {
        // initialize parameters
        double shootPathRadius = teamplay::configurationStore::getConfiguration().getShootPathWidth() / 2.0;
        double lobshot_threshold = 0.6 + ROBOT_RADIUS; // threshold in which obstacles block a lobshot + ROBOT_RADIUS due to the size of the obstacle
        double opp_goalkeeper_positionmargin = 0.5;

        Position2D robotPos2D = teamplay::robotStore::getInstance().getOwnRobot().getPosition();
        Point2D robotPos(robotPos2D.x, robotPos2D.y);

        float preferred_shoottarget_x;
        float preferred_shoottarget_y;
        cWorldStateFunctions::getInstance().getPreferredShootXYOfGoal(preferred_shoottarget_x, preferred_shoottarget_y);
        Point2D preferredPartOfGoal;
        preferredPartOfGoal.x = preferred_shoottarget_x;
        preferredPartOfGoal.y = preferred_shoottarget_y;

        std::vector<robotLocation> obstacles;

        cWorldStateFunctions::getInstance().getObstructingObstaclesInPath(robotPos, preferredPartOfGoal, shootPathRadius, obstacles);

        // obstacles too close to the robot (within lobshot_threshold) assumed to block a lobshot
        for(size_t i = 0; i < obstacles.size(); i++)
        {
            if (calc_distance( obstacles.at(i).position.getX(), obstacles.at(i).position.getY(), robotPos.x, robotPos.y) < lobshot_threshold)
            {
                // Check if opponent is not in opponent goalarea (= opponent goalkeeper)
                if (!cEnvironmentField::getInstance().isPositionInArea(
                        obstacles.at(i).position.getX(), obstacles.at(i).position.getY(),
                        A_OPP_GOALAREA, opp_goalkeeper_positionmargin))
                {
                    retVal = true; // lobshot on goal is blocked by an obstacle
                }
            }
        }

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool isPassToClosestTeammemberBlocked(const std::map<std::string, std::string> &params)
{
    bool retVal = false; // initially pass to closest teammember is not blocked

    try
    {
        double shootPathRadius = teamplay::configurationStore::getConfiguration().getShootPathWidth() / 2.0;

        Position2D robotPos2D = teamplay::robotStore::getInstance().getOwnRobot().getPosition();

        Point2D robotPos(robotPos2D.x, robotPos2D.y);

        // get closest teammember Point2D
        Point2D closest_teammember;
        cWorldStateFunctions::getInstance().getClosestTeammember(closest_teammember.x, closest_teammember.y, false);

        std::vector<robotLocation> obstacles;

        cWorldStateFunctions::getInstance().getObstructingObstaclesInPath(robotPos, closest_teammember, shootPathRadius, obstacles);

        if (obstacles.size() > 0)
        {
            retVal = true; // pass to closest teammember is blocked by an obstacle
        }

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool isPassToClosestAttackerBlocked(const std::map<std::string, std::string> &params)
{
    bool retVal = false; // initially pass to closest attacker is not blocked

    try
    {
        double shootPathRadius = teamplay::configurationStore::getConfiguration().getShootPathWidth() / 2.0;

        Position2D robotPos2D = teamplay::robotStore::getInstance().getOwnRobot().getPosition();

        Point2D robotPos(robotPos2D.x, robotPos2D.y);

        // get closest attacker Point2D
        Point2D closest_attacker;
        bool attackerFound = cWorldStateFunctions::getInstance().getClosestAttacker(A_FIELD, closest_attacker.x, closest_attacker.y);

        if (attackerFound)
        {
            std::vector<robotLocation> obstacles;

            cWorldStateFunctions::getInstance().getObstructingObstaclesInPath(robotPos, closest_attacker, shootPathRadius, obstacles);

            if (obstacles.size() > 0)
            {
                retVal = true; // pass to closest attacker is blocked by an obstacle
            }
        }
        else
        {
            retVal = true; //there is no attacker, so fallback: yes path is blocked
        }

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool isPassToFurthestAttackerBlocked(const std::map<std::string, std::string> &params)
{
    bool retVal = false; // initially pass to closest attacker is not blocked

    try
    {
        double shootPathRadius = teamplay::configurationStore::getConfiguration().getShootPathWidth() / 2.0;

        Position2D robotPos2D = teamplay::robotStore::getInstance().getOwnRobot().getPosition();

        Point2D robotPos(robotPos2D.x, robotPos2D.y);

        // get furthest attacker Point2D
        Point2D furthest_attacker;
        cWorldStateFunctions::getInstance().getClosestAttackerToOpponentGoal(furthest_attacker.x, furthest_attacker.y);

        std::vector<robotLocation> obstacles;

        cWorldStateFunctions::getInstance().getObstructingObstaclesInPath(robotPos, furthest_attacker, shootPathRadius, obstacles);

        if (obstacles.size() > 0)
        {
            retVal = true; // pass to closest attacker is blocked by an obstacle
        }

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool isPassToFurthestDefenderBlocked(const std::map<std::string, std::string> &params)
{
    bool retVal = false; // initially pass to closest defender is not blocked

    try
    {
        double shootPathRadius = teamplay::configurationStore::getConfiguration().getShootPathWidth() / 2.0;

        Position2D robotPos2D = teamplay::robotStore::getInstance().getOwnRobot().getPosition();

        Point2D robotPos(robotPos2D.x, robotPos2D.y);

        // get furthest defender Point2D
        Point2D furthest_defender;
        cWorldStateFunctions::getInstance().getClosestDefenderToOpponentGoal(furthest_defender.x, furthest_defender.y);

        std::vector<robotLocation> obstacles;

        cWorldStateFunctions::getInstance().getObstructingObstaclesInPath(robotPos, furthest_defender, shootPathRadius, obstacles);

        if (obstacles.size() > 0)
        {
            retVal = true; // pass to closest attacker is blocked by an obstacle
        }

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool isTipInBlocked(const std::map<std::string, std::string> &params)
{
    bool retVal = true; // initially pass to Tip-In position is blocked, due to a pass with higher risk

    try
    {
        Point2D tipInPOI = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::TIP_IN);
        double shootPathRadius = teamplay::configurationStore::getConfiguration().getShootPathWidth() / 2.0;

        Position2D robotPos2D = teamplay::robotStore::getInstance().getOwnRobot().getPosition();
        Point2D robotPos(robotPos2D.x, robotPos2D.y);

        // check if obstacles on path to tipInPOI
        std::vector<robotLocation> obstacles;

        cWorldStateFunctions::getInstance().getObstructingObstaclesInPath(robotPos, tipInPOI, shootPathRadius, obstacles);

        if (obstacles.size() == 0)
        {
            retVal = false; // pass to tipIn position is not blocked by an obstacle
        }

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool isPathToBallBlocked(const std::map<std::string, std::string> &params)
{
    bool retVal = false; // initially path between ball and robot is not blocked

    try
    {
        double shootPathRadius = teamplay::configurationStore::getConfiguration().getShootPathWidth() / 2.0;

        Position2D robotPos2D = teamplay::robotStore::getInstance().getOwnRobot().getPosition();
        Point2D robotPos(robotPos2D.x, robotPos2D.y);

        // get ball data and position
        teamplay::ball ball = teamplay::ballStore::getBall();
        Point2D ballPos(ball.getPosition().x, ball.getPosition().y);

        // check any obstacle obstacles
        std::vector<robotLocation> obstacles;

        cWorldStateFunctions::getInstance().getObstructingObstaclesInPath(robotPos, ballPos, shootPathRadius, obstacles);

        if (obstacles.size() > 0)
        {
            retVal = true; // path between ball and robot is blocked by an obstacle
        }

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool doesAssistantHaveBall(const std::map<std::string, std::string> &params)
{
    try
    {
        auto own_assistant = teamplay::robotStore::getInstance().getAssistantOfOwnRobot();

        if (own_assistant)
        {
            return own_assistant->hasBall();
        }
        else
        {
            return false;
        }
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool allRobotsActive(const std::map<std::string, std::string> &params)
{
    bool retVal = false;

    try
    {
        retVal = (teamplay::robotStore::getInstance().getAllRobots().size() == 5);
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool defendingStrategyOn(const std::map<std::string, std::string> &params)
{
    bool retVal = false;

    try
    {
        retVal = teamplay::configurationStore::getConfiguration().getDefendingStrategy();
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool dribbleStrategyOn(const std::map<std::string, std::string> &params)
{
    bool retVal = false;

    try
    {
        retVal = teamplay::configurationStore::getConfiguration().getDribbleStrategy();
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool multipleOpponentsOnOwnHalf(const std::map<std::string, std::string> &params)
{
    bool retVal = false;

    try
    {
        int nrOfOpponentsOnOwnHalf = 0;

        auto opponents = teamplay::obstacleStore::getInstance().getAllObstacles();

        for (auto opponent = opponents.begin(); opponent != opponents.end(); opponent++)
        {
            auto opp_location = opponent->getLocation();

            if (opp_location.y < 0.0)
            {
                nrOfOpponentsOnOwnHalf = nrOfOpponentsOnOwnHalf + 1;
            }
        }

        retVal = (nrOfOpponentsOnOwnHalf > 1);

    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool isAnAttackerOnOppHalf(const std::map<std::string, std::string> &params)
{
    bool retVal = false;

    try
    {
        // get closest attacker Point2D
        Point2D closest_attacker(0.0, 0.0);
        retVal = cWorldStateFunctions::getInstance().getClosestAttacker(A_OPP_SIDE, closest_attacker.x, closest_attacker.y);
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}


bool shotThresholdReached(const std::map<std::string, std::string> &params)
{
    bool retVal = false;

    try
    {
        float shotThreshold = teamplay::configurationStore::getConfiguration().getShotThreshold();

        Point2D ownPos = teamplay::robotStore::getInstance().getOwnRobot().getLocation();

        Point2D opp_goalline_center;
        opp_goalline_center = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OPP_GOALLINE_CENTER);

        retVal = ( calc_distance(ownPos.x, ownPos.y, opp_goalline_center.x, opp_goalline_center.y) < shotThreshold );

    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool shotThresholdReachable(const std::map<std::string, std::string> &params)
{
    bool retVal = false;

    try
    {
        float shotThreshold = teamplay::configurationStore::getConfiguration().getShotThreshold();
        float distToShotThreshold = 2.0;

        Point2D ownPos = teamplay::robotStore::getInstance().getOwnRobot().getLocation();

        Point2D opp_goalline_center;
        opp_goalline_center = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OPP_GOALLINE_CENTER);

        retVal = ( calc_distance(ownPos.x, ownPos.y, opp_goalline_center.x, opp_goalline_center.y) < (shotThreshold + distToShotThreshold));

    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

bool cWorldStateFunctions::getPreferredShootXYOfGoal(float &target_x, float &target_y)
{
    bool retVal = true; // always return preferred part of goal, default middle of goal

    try
    {
        // own position
        Position2D myPos = teamplay::robotStore::getInstance().getOwnRobot().getPosition();

        // some variables that read nice
        auto goalCenter = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OPP_GOALLINE_CENTER);
        auto goalPostLeft = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OPP_GOALPOST_LEFT);
        auto goalPostRight = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OPP_GOALPOST_RIGHT);

        double goalPostShootOffset = 0.375; // the robot should shoot inside the goal area i.o. onto the goalpost
        goalPostLeft.x = goalPostLeft.x + goalPostShootOffset;
        goalPostRight.x = goalPostRight.x - goalPostShootOffset;

        // default aim is opponent goalline center
        auto preferredPartOfGoal = goalCenter;

        // only take opp goalkeeper into account when close enough to goal (due to shot inaccuracy long distances)
        std::map<std::string, std::string> params;
        if ( isOpponentGoalKeeperInLeftCorner(params) )
        {
            // Opponent goalkeeper is positioned in left corner, shoot in right corner
            preferredPartOfGoal = goalPostRight;
        }
        else if ( isOpponentGoalKeeperInRightCorner(params) )
        {
            // Opponent goalkeeper is positioned in right corner, shoot in left corner
            preferredPartOfGoal = goalPostLeft;
        }
        else
        {
            // is the robot positioned on left side of field, shoot right corner
            if (teamplay::fieldDimensionsStore::getFieldDimensions().isPositionInLeftSide(myPos.x, myPos.y))
            {
                preferredPartOfGoal = goalPostRight;
            }
            else
            {
                // else shoot in left corner
                preferredPartOfGoal = goalPostLeft;
            }
        }

        target_x = preferredPartOfGoal.x;
        target_y = preferredPartOfGoal.y;

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return retVal;
}

boost::optional<Position2D> cWorldStateFunctions::getPos2DFromStr(const std::map<std::string, std::string> &parameters, std::string &param)
{
    Position2D retVal;

    auto paramVal = parameters.find(param);
    if (paramVal != parameters.end())
    {
        std::string strParam = paramVal->second;
        return cWorldStateFunctions::getInstance().getPositionOfPOI(strParam);
    }

    return boost::none;
}

boost::optional<Position2D> cWorldStateFunctions::getPositionOfPOI(const std::string POI) const
{
    Position2D retVal;

    // If param is the empty value, return boost::none;
    if (POI.compare(emptyValue) == 0)
    {
        return boost::none;
    }
    // If param has value 'ball', return ballposition (if possible)
    else if (POI.compare("ball") == 0)
    {
        auto ball = teamplay::ballStore::getBall();

        if (ball.isLocationKnown())
        {
            retVal.x = ball.getPosition().x;
            retVal.y = ball.getPosition().y;
            return retVal;
        }
    }
    else if(POI.compare("lastKnownBallLocation") == 0)
    {
        Point3D ball = teamplay::ballStore::getBall().getPosition();

        retVal.x = ball.x;
        retVal.y = ball.y;
        return retVal;
    }
    else if (POI.compare("robot") == 0)
    {
        Position2D myPos = teamplay::robotStore::getInstance().getOwnRobot().getPosition();

        retVal.x = myPos.x;
        retVal.y = myPos.y;
        return retVal;
    }
    else if (POI.compare("closestTeammemberIncludeGoalie") == 0)
    {
        Position2D teammemberPos;
        cWorldStateFunctions::getInstance().getClosestTeammember(teammemberPos.x, teammemberPos.y, true);

        retVal.x = teammemberPos.x;
        retVal.y = teammemberPos.y;
        return retVal;
    }
    else if (POI.compare("closestTeammember") == 0)
    {
        Position2D teammemberPos;
        cWorldStateFunctions::getInstance().getClosestTeammember(teammemberPos.x, teammemberPos.y, false);

        retVal.x = teammemberPos.x;
        retVal.y = teammemberPos.y;
        return retVal;
    }
    else if (POI.compare("closestAttacker") == 0)
    {
        double attackerPosX = 0.0;
        double attackerPosY = 0.0;
        cWorldStateFunctions::getInstance().getClosestAttacker(A_FIELD, attackerPosX, attackerPosY);

        retVal.x = attackerPosX;
        retVal.y = attackerPosY;
        return retVal;
    }
    else if (POI.compare("closestAttackerOnOppHalf") == 0)
    {
        double attackerPosX = 0.0;
        double attackerPosY = 0.0;
        cWorldStateFunctions::getInstance().getClosestAttacker(A_OPP_SIDE, attackerPosX, attackerPosY);

        retVal.x = attackerPosX;
        retVal.y = attackerPosY;
        return retVal;
    }
    else if (POI.compare("closestAttackerToOppGoal") == 0)
    {
        double attackerPosX = 0.0;
        double attackerPosY = 0.0;
        cWorldStateFunctions::getInstance().getClosestAttackerToOpponentGoal(attackerPosX, attackerPosY);

        retVal.x = attackerPosX;
        retVal.y = attackerPosY;
        return retVal;
    }
    else if (POI.compare("closestDefender") == 0)
    {
        double defenderPosX = 0.0;
        double defenderPosY = 0.0;
        cWorldStateFunctions::getInstance().getClosestDefender(defenderPosX, defenderPosY);

        retVal.x = defenderPosX;
        retVal.y = defenderPosY;
        return retVal;
    }
    else if (POI.compare("closestDefenderToOppGoal") == 0)
    {
        double defenderPosX = 0.0;
        double defenderPosY = 0.0;
        cWorldStateFunctions::getInstance().getClosestDefenderToOpponentGoal(defenderPosX, defenderPosY);

        retVal.x = defenderPosX;
        retVal.y = defenderPosY;
        return retVal;
    }
    else if (POI.compare("closestOpponent") == 0)
    {
        float opponentPosX = 0.0;
        float opponentPosY = 0.0;
        cWorldStateFunctions::getInstance().getClosestOpponent(opponentPosX, opponentPosY);

        retVal.x = opponentPosX;
        retVal.y = opponentPosY;
        return retVal;
    }
    else if (POI.compare("closestOpponentToOwnGoal") == 0)
    {
        Point2D own_goalline_center = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OWN_GOALLINE_CENTER);
        float opponentPosX = 0.0;
        float opponentPosY = 0.0;
        cWorldStateFunctions::getInstance().getClosestOpponentToLocationXY(opponentPosX, opponentPosY, (float) own_goalline_center.x, (float) own_goalline_center.y);

        retVal.x = opponentPosX;
        retVal.y = opponentPosY;
        return retVal;
    }
    else if (POI.compare("potentialOppAttacker") == 0)
    {
        float opponentAttackerPosX = 0.0;
        float opponentAttackerPosY = 0.0;
        cWorldStateFunctions::getInstance().getPotentialOpponentAttacker(opponentAttackerPosX, opponentAttackerPosY);

        retVal.x = opponentAttackerPosX;
        retVal.y = opponentAttackerPosY;
        return retVal;
    }
    else if( boost::starts_with( POI, "coord:"))
    {   //expected syntax example:  coord 4.5,6.8
        std::vector<std::string> splittedString;
        boost::split( splittedString, POI, boost::is_any_of(":,"));
        if ( splittedString.size() != 3)
        {
            TRACE("MALFORMED coord parameter encountered: ") << POI;
            return boost::none;
        }

        try
        {
           retVal.x = boost::lexical_cast<double>( splittedString[1]);
        }
        catch (std::exception &e)
        {
            TRACE("MALFORMED coord parameter encountered for X in: ") << POI;
            return boost::none;
        }

        try
        {
           retVal.y = boost::lexical_cast<double>( splittedString[2]);
        }
        catch (std::exception &e)
        {
            TRACE("MALFORMED coord parameter encountered for Y in: ") << POI;
            return boost::none;
        }
        return retVal;
    }
    else
    {
        try
        {
            Point2D paramInfo = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(POI);
            retVal.x = paramInfo.x;
            retVal.y = paramInfo.y;
        }
        catch (std::exception &e)
        {
            TRACE("MALFORMED location parameter encountered: ") << POI;
            return boost::none;
        }
        return retVal;
    }

    return boost::none;
}

// worldstatefunctions::getPreferredShotLocationInGoal
// TODO: implement getPreferredShotLocationInGoal in
// TODO: Lobshot line_y > aiming center goal opschuiven
