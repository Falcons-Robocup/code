 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
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

#include "int/stores/ballStore.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/cTeammateInterface.hpp"
#include "int/cTeamplayCommon.hpp"
#include "int/cWorldModelInterface.hpp"
#include "int/rules/ruleStimulatePassing.hpp"
#include "int/stores/fieldDimensionsStore.hpp"
#include "int/stores/gameStateStore.hpp"
#include "int/stores/ownRobotStore.hpp"
#include "int/stores/teamMatesStore.hpp"
#include "int/utilities/trace.hpp"

#include <FalconsCommon.h>

using std::exception;
using std::runtime_error;


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

bool isShortTurnToGoalBlockedByOpponent()
{
    try
    {
        Position2D own_location;
        cWorldModelInterface::getInstance().getOwnLocation(own_location);

        Point2D opp_goalline_center;
        opp_goalline_center = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OPP_GOALLINE_CENTER);

        // Get angle to goal, normalized to [-pi, pi]
        double target_angle = angle_between_two_points_0_2pi(own_location.x, own_location.y, opp_goalline_center.x, opp_goalline_center.y) - own_location.phi;
        target_angle = project_angle_mpi_pi(target_angle);
        TRACE("target angle robot to goal: %6.2f" ,target_angle);

        robotLocations opponents;
        cWorldModelInterface::getInstance().getOpponents(opponents);

        for (auto opponent = opponents.begin(); opponent != opponents.end(); opponent++)
        {
            geometry::Pose2D opp_location = opponent->second.position;
            if (calc_distance(own_location.x, own_location.y, opp_location.x, opp_location.y) < 1.0)
            {
                TRACE("opponent at (%6.2f, %6.2f) is within 1 meter" ,opp_location.x, opp_location.y);

                // The opponent is within 1 meter: get angle to opponent, normalized to [-pi, pi]
                double angle_to_opp = angle_between_two_points_0_2pi(own_location.x, own_location.y, opp_location.x, opp_location.y) - own_location.phi;
                angle_to_opp = project_angle_mpi_pi(angle_to_opp);
                TRACE("angle between robot and opponent: %6.2f", angle_to_opp);

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

bool isLongTurnToGoalBlockedByOpponent()
{
    try
    {
        Position2D own_location;
        cWorldModelInterface::getInstance().getOwnLocation(own_location);

        Point2D opp_goalline_center;
        opp_goalline_center = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OPP_GOALLINE_CENTER);

        // Get angle to goal, normalized to [-pi, pi]
        double target_angle = angle_between_two_points_0_2pi(own_location.x, own_location.y, opp_goalline_center.x, opp_goalline_center.y) - own_location.phi;
        target_angle = project_angle_mpi_pi(target_angle);
        TRACE("target angle robot to goal: %6.2f", target_angle);

        robotLocations opponents;
        cWorldModelInterface::getInstance().getOpponents(opponents);

        for (auto opponent = opponents.begin(); opponent != opponents.end(); opponent++)
        {
            geometry::Pose2D opp_location = opponent->second.position;
            if (calc_distance(own_location.x, own_location.y, opp_location.x, opp_location.y) < 1.0)
            {
                TRACE("opponent at (%6.2f, %6.2f) is within 1 meter", opp_location.x, opp_location.y);

                // The opponent is within 1 meter: get angle to opponent, normalized to [-pi, pi]
                double angle_to_opp = angle_between_two_points_0_2pi(own_location.x, own_location.y, opp_location.x, opp_location.y) - own_location.phi;
                angle_to_opp = project_angle_mpi_pi(angle_to_opp);
                TRACE("angle between robot and opponent: %6.2f", angle_to_opp);

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

bool isShotAtGoalAllowed()
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

bool ballPickupOnOpponentHalf()
{
    try
    {
        return teamplay::ballStore::getBall().isClaimedOnOpponentHalf();
    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool isBallInOwnPenaltyArea()
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

bool isMemberInOwnPenaltyArea()
{
	bool retVal = false;

	try
	{
		retVal = cWorldStateFunctions::getInstance().isMemberInArea(A_OWN_PENALTYAREA, true, true);
	} catch (exception &e)
	{
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
	}

	return retVal;
}


bool isBallInOpponentPenaltyArea()
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

bool isMemberInOpponentPenaltyArea()
{
	bool ret_val = false;

	try
	{
		ret_val = cWorldStateFunctions::getInstance().isMemberInArea(A_OPP_PENALTYAREA, true, true);
	} catch (exception &e)
	{
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
	return ret_val;
}

bool cWorldStateFunctions::isMemberInArea(areaName area, bool includeOwnRobot, bool includeGoalie)
{
	bool retVal = false;

	try
	{
		// get all team mates locations from WorldModelInterface
	    robotLocations teamMembers;
		cWorldModelInterface::getInstance().getTeammembers(teamMembers);

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

bool doesTeamHaveBall()
{
	bool ret_val = false;

	try
	{
		/* Check own ball possession */
		ballPossession_struct_t ball_possession;
		cWorldModelInterface::getInstance().getBallPossession(ball_possession);

		if ((ball_possession.possessionType == ballPossessionEnum::TEAMMEMBER) )
		{
			ret_val = true;
		}
	} catch (exception &e)
	{
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
	}

	return ret_val;
}

bool doesOpponentHaveBall()
{
	bool ret_val = false;

	try
	{
		/*
		 * Opponent cannot have ball anymore, only field or team member
		 */
	} catch (exception &e)
	{
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
	}

	return ret_val;
}

bool doesNoTeamHaveBall()
{
	bool ret_val = false;

	try
	{
		/* Check own ball possession */
		ballPossession_struct_t ball_possession;
		cWorldModelInterface::getInstance().getBallPossession(ball_possession);

		if ((ball_possession.possessionType == ballPossessionEnum::FIELD) )
		{
			ret_val = true;
		}
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

        // Get robot position
        robotLocations teamMembers;
        cWorldModelInterface::getInstance().getTeammembers(teamMembers);

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
		robotLocations teamMembers;
		cWorldModelInterface::getInstance().getTeammembers(teamMembers);

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
	    robotLocations teamMembers;
		cWorldModelInterface::getInstance().getTeammembers(teamMembers);
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

void cWorldStateFunctions::getClosestAttacker(areaName area, double &target_x, double &target_y)
{
    try
    {

        // get my own location
        Position2D myPos = getLocationOfRobot(_robotID);

        double distanceToClosestAttacker = 999.0;

        // Find attackers.
        boost::optional<robotNumber> attackerMain = getRobotWithRole(treeEnum::ATTACKER_MAIN);
        boost::optional<robotNumber> attackerAssist = getRobotWithRole(treeEnum::ATTACKER_ASSIST);

        robotLocations teamMembers;
        cWorldModelInterface::getInstance().getTeammembers(teamMembers);

        // TODO: what if no attacker found in area??? this was already the case with 1 attacker-main and no attacker-assist

        if (attackerMain &&
    	(*attackerMain != _robotID) &&
    	(teamMembers.find(*attackerMain) != teamMembers.end()))// If attackerMain role exists, and it's not me
        {
            // Compute distance between attacker and me.
            Position2D robotPos = getPosition2D(teamMembers.at(*attackerMain).position);

            // check current team member coordinates against the area match
    		if (cEnvironmentField::getInstance().isPositionInArea((float) robotPos.x, (float) robotPos.y, area,	_positionMargin))
    		{
				double distance = calc_distance( myPos, robotPos );
				if (distance < distanceToClosestAttacker)
				{
					distanceToClosestAttacker = distance;
					target_x = robotPos.x;
					target_y = robotPos.y;
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
    		if (cEnvironmentField::getInstance().isPositionInArea((float) robotPos.x, (float) robotPos.y, area,	_positionMargin))
    		{
                double distance = calc_distance( myPos, robotPos );
                if (distance < distanceToClosestAttacker)
                {
                    distanceToClosestAttacker = distance;
                    target_x = robotPos.x;
                    target_y = robotPos.y;
                }
    		}
        }

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

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

        robotLocations teamMembers;
        cWorldModelInterface::getInstance().getTeammembers(teamMembers);

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

        robotLocations teamMembers;
        cWorldModelInterface::getInstance().getTeammembers(teamMembers);

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
		robotLocations opponents;
		cWorldModelInterface::getInstance().getOpponents( opponents );

		std::vector<robotLocation> robots;
		for(auto it = opponents.begin(); it != opponents.end(); it++)
		{
			robots.push_back(it->second);
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
		robotLocations opponents;
		cWorldModelInterface::getInstance().getOpponents( opponents );

		std::vector<robotLocation> robots;
		for(auto it = opponents.begin(); it != opponents.end(); it++)
		{
			robots.push_back(it->second);
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
        std::vector<robotNumber> active_robots;
        cWorldModelInterface::getInstance().getActiveRobots(active_robots);

        std::vector<robotNumber>::const_iterator it;
        for (it = active_robots.begin(); it != active_robots.end(); ++it)
        {
            treeEnum robotRole = getRobotRole(*it);

            if (robotRole == role)
            {
                return *it;
            }
        }
    }
    catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }

    return boost::none;
}


bool isLowestActiveRobotID()
{
    try
    {
        auto own_robot_id = teamplay::ownRobotStore::getOwnRobot().getNumber();
        auto teammates = teamplay::teamMatesStore::getInstance().getTeamMatesIncludingGoalie();

        return (teammates.getNumberOfRobotsWithIDLowerThan(own_robot_id) == 0);
    }
    catch (exception &e)
    {
        TRACE_ERROR(e.what());
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool isHighestActiveRobotID()
{
    try
    {
        auto own_robot_id = teamplay::ownRobotStore::getOwnRobot().getNumber();
        auto teammates = teamplay::teamMatesStore::getInstance().getTeamMatesIncludingGoalie();

        return (teammates.getNumberOfRobotsWithIDHigherThan(own_robot_id) == 0);
    }
    catch (exception &e)
    {
        TRACE_ERROR(e.what());
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool isSecondHighestActiveRobotID()
{
    try
    {
        auto own_robot_id = teamplay::ownRobotStore::getOwnRobot().getNumber();
        auto teammates = teamplay::teamMatesStore::getInstance().getTeamMatesIncludingGoalie();

        return (teammates.getNumberOfRobotsWithIDHigherThan(own_robot_id) == 1);
    }
    catch (exception &e)
    {
        TRACE_ERROR(e.what());
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool isThirdHighestActiveRobotID()
{
    try
    {
        auto own_robot_id = teamplay::ownRobotStore::getOwnRobot().getNumber();
        auto teammates = teamplay::teamMatesStore::getInstance().getTeamMatesIncludingGoalie();

        return (teammates.getNumberOfRobotsWithIDHigherThan(own_robot_id) == 2);
    }
    catch (exception &e)
    {
        TRACE_ERROR(e.what());
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool isOnlyActiveRobotID()
{
    try
    {
        auto teammates = teamplay::teamMatesStore::getInstance().getTeamMatesIncludingGoalie();

        return (teammates.getNumberOfRobots() == 0);
    }
    catch (exception &e)
    {
        TRACE_ERROR(e.what());
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool returnTrue()
{
    return true;
}

bool returnFalse()
{
    return false;
}

bool isInMatch()
{
    return teamplay::gameStateStore::getInstance().getGameState().isInMatch();
}

bool isSetPiece()
{
    return teamplay::gameStateStore::getInstance().getGameState().isSetPiece();
}

bool isOwnSetPiece()
{
    return teamplay::gameStateStore::getInstance().getGameState().isOwnSetPiece();
}

bool isPrepareSetPiece()
{
    return teamplay::gameStateStore::getInstance().getGameState().isPrepareSetPiece();
}

bool isKickoffSetPiece()
{
    return teamplay::gameStateStore::getInstance().getGameState().isKickoffSetPiece();
}

bool isDroppedBallSetPiece()
{
    return teamplay::gameStateStore::getInstance().getGameState().isDroppedBallSetPiece();
}

bool isSidelineSetPiece()
{
    return teamplay::gameStateStore::getInstance().getGameState().isSidelineSetPiece();
}

bool isSidelineSetPieceRight()
{
    return (isSidelineSetPiece() && teamplay::ballStore::getBall().isAtRightSide());
}

bool isPenaltySetPiece()
{
    return teamplay::gameStateStore::getInstance().getGameState().isPenaltySetPiece();
}

bool isGoalkickSetPiece()
{
    return teamplay::gameStateStore::getInstance().getGameState().isGoalkickSetPiece();
}

bool isFreekickSetPiece()
{
    return teamplay::gameStateStore::getInstance().getGameState().isFreekickSetPiece();
}

bool isCornerSetPiece()
{
    return teamplay::gameStateStore::getInstance().getGameState().isCornerSetPiece();
}

bool isThrowinSetPiece()
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


bool within1mOfBall()
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

bool doesOwnRobotHaveBall()
{
	bool retVal = false;

	try
	{
		/* Check own ball possession */
		ballPossession_struct_t ball_possession;
		cWorldModelInterface::getInstance().getBallPossession(ball_possession);

		if ((ball_possession.possessionType == ballPossessionEnum::TEAMMEMBER)
				&& (ball_possession.robotID == cWorldStateFunctions::getInstance().getRobotID()))
		{
			/* if robot has the ball, return true */
			retVal = true;
		}
	}
	catch (exception &e)
	{
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
	}

	return retVal;
}

bool isOwnRobotAtOpponentSide()
{
	try
	{
		Position2D ownPos;
		cWorldModelInterface::getInstance().getOwnLocation(ownPos);
		return teamplay::fieldDimensionsStore::getFieldDimensions().isPositionInOpponentSide(ownPos.x, ownPos.y);

	}
	catch (exception &e)
	{
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

bool isBallAtOpponentSide()
{
	try
	{
        return teamplay::ballStore::getBall().isAtOpponentSide();
	}
	catch (exception &e)
	{
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

bool isBallAtOwnSide()
{
	try
	{
        return teamplay::ballStore::getBall().isAtOwnSide();
	}
	catch (exception &e)
	{
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

bool isBallAtLeftSide()
{
	try
	{
        return teamplay::ballStore::getBall().isAtLeftSide();
	}
	catch (exception &e)
	{
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

bool isBallAtRightSide()
{
	try
	{
        return teamplay::ballStore::getBall().isAtRightSide();
	}
	catch (exception &e)
	{
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

bool isOwnRobotNearestToBall()
{
	bool retVal = false;

	try
	{
		uint8_t robotID = 0;
		bool memberFound = false;
		teamplay::ball ball = teamplay::ballStore::getBall();

		if(ball.isLocationKnown())
		{
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
	}
	catch (exception &e)
	{
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
	}

	return retVal;
}

bool isOwnRobotNearestToLastKnownBallLocation()
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

bool isBallLocationKnown()
{
    return teamplay::ballStore::getBall().isLocationKnown();
}

/*!
 * \brief Is my assistant present?
 *
 * ATTACKER_MAIN checks for ATTACKER_ASSIST and vice versa
 * DEFENDER_MAIN checks for DEFENDER_ASSIST and vice versa
 * All others return true, except for ROBOT_STOP, which returns false.
 */
bool isAssistentPresent()
{
    auto myRole = teamplay::ownRobotStore::getOwnRobot().getRole();
    return teamplay::teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(myRole);
}

/*!
 * \brief Am I the closest attacker to the ball?
 * Returns false if I am not an attacker.
 * Returns true if there is no other attacker present.
 */
bool isClosestAttackerToBall()
{
    bool retVal = false;

    // Determine my role
    robotNumber myNr = cWorldStateFunctions::getInstance().getRobotID();
    treeEnum myRole = cWorldStateFunctions::getInstance().getRobotRole(myNr);

    // First check if I am an attacker.
    switch(myRole)
    {
        case treeEnum::ATTACKER_MAIN:
        {
            try
            {
                // Find ATTACKER_ASSIST, see if he is closer to the ball than me.
                boost::optional<robotNumber> attackerAssist = cWorldStateFunctions::getInstance().getRobotWithRole(treeEnum::ATTACKER_ASSIST);

                if (attackerAssist)
                {
                    // Assistant found. Am I closer to the ball?

                    boost::optional<float> myDistToBall = cWorldStateFunctions::getInstance().ballDistance(myNr);
                    boost::optional<float> assistentDistToBall = cWorldStateFunctions::getInstance().ballDistance(*attackerAssist);

                    if (myDistToBall && assistentDistToBall)
                    {
                        if (*myDistToBall < *assistentDistToBall)
                        {
                            retVal = true;
                        }
                    }
                }
                else
                {
                    // If no assistant found, then I am definitely the closest attacker
                    retVal = true;
                }
            }
            catch (exception &e)
            {
                std::cout << "Caught exception: " << e.what() << std::endl;
                throw std::runtime_error(std::string("Linked to: ") + e.what());
            }
            break;
        }
        case treeEnum::ATTACKER_ASSIST:
        {
            try
            {
                // Find ATTACKER_MAIN, see if he is closer to the ball than me.
                boost::optional<robotNumber> attackerMain = cWorldStateFunctions::getInstance().getRobotWithRole(treeEnum::ATTACKER_MAIN);

                if (attackerMain)
                {
                    // Main found. Am I closer to the ball?

                    boost::optional<float> myDistToBall = cWorldStateFunctions::getInstance().ballDistance(myNr);
                    boost::optional<float> mainDistToBall = cWorldStateFunctions::getInstance().ballDistance(*attackerMain);

                    if (myDistToBall && mainDistToBall)
                    {
                        if (*myDistToBall < *mainDistToBall)
                        {
                            retVal = true;
                        }
                    }
                }
                else
                {
                    // If main was not found (??) then I am the only attacker, thus the closest to the ball.
                    retVal = true;
                }
            }
            catch (exception &e)
            {
                std::cout << "Caught exception: " << e.what() << std::endl;
                throw std::runtime_error(std::string("Linked to: ") + e.what());
            }
            break;
        }
        default:
        {
            // I am not an attacker. Return false.
            retVal = false;
            break;
        }
    }

    return retVal;
}

bool isClosestDefenderToBall()
{
    bool retVal = false;

    // Determine my role
    robotNumber myNr = cWorldStateFunctions::getInstance().getRobotID();
    treeEnum myRole = cWorldStateFunctions::getInstance().getRobotRole(myNr);

    // First check if I am a defender.
    switch(myRole)
    {
        case treeEnum::DEFENDER_MAIN:
        {
            try
            {
                // Find DEFENDER_ASSIST, see if he is closer to the ball than me.
                boost::optional<robotNumber> defenderAssist = cWorldStateFunctions::getInstance().getRobotWithRole(treeEnum::DEFENDER_ASSIST);

                if (defenderAssist)
                {
                    // Assistant found. Am I closer to the ball?

                    boost::optional<float> myDistToBall = cWorldStateFunctions::getInstance().ballDistance(myNr);
                    boost::optional<float> assistentDistToBall = cWorldStateFunctions::getInstance().ballDistance(*defenderAssist);

                    if (myDistToBall && assistentDistToBall)
                    {
                        if (*myDistToBall < *assistentDistToBall)
                        {
                            retVal = true;
                        }
                    }
                }
                else
                {
                    // If no assistant found, then I am definitely the closest defender
                    retVal = true;
                }
            }
            catch (exception &e)
            {
                std::cout << "Caught exception: " << e.what() << std::endl;
                throw std::runtime_error(std::string("Linked to: ") + e.what());
            }
            break;
        }
        case treeEnum::DEFENDER_ASSIST:
        {
            try
            {
                // Find DEFENDER_MAIN, see if he is closer to the ball than me.
                boost::optional<robotNumber> defenderMain = cWorldStateFunctions::getInstance().getRobotWithRole(treeEnum::DEFENDER_MAIN);

                if (defenderMain)
                {
                    // Main found. Am I closer to the ball?

                    boost::optional<float> myDistToBall = cWorldStateFunctions::getInstance().ballDistance(myNr);
                    boost::optional<float> mainDistToBall = cWorldStateFunctions::getInstance().ballDistance(*defenderMain);

                    if (myDistToBall && mainDistToBall)
                    {
                        if (*myDistToBall < *mainDistToBall)
                        {
                            retVal = true;
                        }
                    }
                }
                else
                {
                    // If main was not found (??) then I am the only defender, thus the closest to the ball.
                    retVal = true;
                }
            }
            catch (exception &e)
            {
                std::cout << "Caught exception: " << e.what() << std::endl;
                throw std::runtime_error(std::string("Linked to: ") + e.what());
            }
            break;
        }
        default:
        {
            // I am not a defender. Return false.
            retVal = false;
            break;
        }
    }

    return retVal;
}

bool isPotentialOppAttackerPresent()
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
        Position2D ownPos;
        cWorldModelInterface::getInstance().getOwnLocation(ownPos);

        return ownPos;
    }
    else
    {
        // Otherwise, return the given robot's location
        robotLocations teamMembers;
        cWorldModelInterface::getInstance().getTeammembers(teamMembers);

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

bool isOpponentGoalKeeperInLeftCorner()
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

bool isOpponentGoalKeeperInRightCorner()
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

bool isBallApproachingRobot()
{
    try
    {
        auto captureRadius = teamplay::configurationStore::getConfiguration().getInterceptBallCaptureRadius();
        auto minimumSpeed = teamplay::configurationStore::getConfiguration().getInterceptBallMinimumSpeed();

        // get robot position
        Position2D currentPos;
		cWorldModelInterface::getInstance().getOwnLocation(currentPos);
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

        return (ballIntersectsCaptureRadius && ballIsMovingTowardsUs && ballMovingFastEnough);

    } catch (exception &e)
    {
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

bool isOpponentHalfReachable()
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
		robotLocations teammembers;
		cWorldModelInterface::getInstance().getTeammembers(teammembers);

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
		robotLocations opponents;
		cWorldModelInterface::getInstance().getOpponents(opponents);

		for(robotLocations::iterator i = opponents.begin(); i != opponents.end(); i++)
		{
			robotLocation obstacle = i->second;
			inputSet.push_back(obstacle);
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

bool isShotOnGoalBlocked()
{
	bool retVal = false; // initially shot on goal is not blocked

    try
    {
        double shootPathRadius = teamplay::configurationStore::getConfiguration().getShootPathWidth() / 2.0;
    	double opp_goalkeeper_positionmargin = 0.5;

    	Position2D robotPos2D;
    	cWorldModelInterface::getInstance().getOwnLocation(robotPos2D);
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

bool isLobShotOnGoalBlocked()
{
	bool retVal = false; // initially shot on goal is not blocked

    try
    {
    	// initialize parameters
        double shootPathRadius = teamplay::configurationStore::getConfiguration().getShootPathWidth() / 2.0;
    	double lobshot_threshold = 0.2; // threshold in which obstacles block a lobshot (within 1.25m)
    	double opp_goalkeeper_positionmargin = 0.5;

    	Position2D robotPos2D;
    	cWorldModelInterface::getInstance().getOwnLocation(robotPos2D);
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

bool isPassToClosestTeammemberBlocked()
{
	bool retVal = false; // initially pass to closest teammember is not blocked

    try
    {
        double shootPathRadius = teamplay::configurationStore::getConfiguration().getShootPathWidth() / 2.0;

    	Position2D robotPos2D;
    	cWorldModelInterface::getInstance().getOwnLocation(robotPos2D);

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

bool isPassToClosestAttackerBlocked()
{
	bool retVal = false; // initially pass to closest attacker is not blocked

    try
    {
        double shootPathRadius = teamplay::configurationStore::getConfiguration().getShootPathWidth() / 2.0;

    	Position2D robotPos2D;
    	cWorldModelInterface::getInstance().getOwnLocation(robotPos2D);

    	Point2D robotPos(robotPos2D.x, robotPos2D.y);

    	// get closest attacker Point2D
    	Point2D closest_attacker;
    	cWorldStateFunctions::getInstance().getClosestAttacker(A_FIELD, closest_attacker.x, closest_attacker.y);

    	std::vector<robotLocation> obstacles;

        cWorldStateFunctions::getInstance().getObstructingObstaclesInPath(robotPos, closest_attacker, shootPathRadius, obstacles);

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

bool isPassToFurthestAttackerBlocked()
{
	bool retVal = false; // initially pass to closest attacker is not blocked

    try
    {
        double shootPathRadius = teamplay::configurationStore::getConfiguration().getShootPathWidth() / 2.0;

    	Position2D robotPos2D;
    	cWorldModelInterface::getInstance().getOwnLocation(robotPos2D);

    	Point2D robotPos(robotPos2D.x, robotPos2D.y);

    	// get closest attacker Point2D
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

bool isTipInBlocked()
{
	bool retVal = true; // initially pass to Tip-In position is blocked, due to a pass with higher risk

    try
    {
        double shootPathRadius = teamplay::configurationStore::getConfiguration().getShootPathWidth() / 2.0;

    	Position2D robotPos2D;
    	cWorldModelInterface::getInstance().getOwnLocation(robotPos2D);

    	Point2D robotPos(robotPos2D.x, robotPos2D.y);

    	// get tipIn Point2D
    	Point2D tipInPOI;
    	tipInPOI.x = 0.75; //0.75 [m] from middle line, opposite half of were ball is positioned
    	tipInPOI.y = 7.0; // 2.0 [m] from opponent goal

    	if (teamplay::ballStore::getBall().isAtRightSide())
    	{
    		tipInPOI.x = -tipInPOI.x; // tipIn position opposite side of were ball is positioned
    	}

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

bool isPathToBallBlocked()
{
	bool retVal = false; // initially path between ball and robot is not blocked

    try
    {
        double shootPathRadius = teamplay::configurationStore::getConfiguration().getShootPathWidth() / 2.0;

    	Position2D robotPos2D;
    	cWorldModelInterface::getInstance().getOwnLocation(robotPos2D);
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

bool doesAssistantHaveBall()
{
	bool retVal = false;

	try
	{
	    auto myRole = teamplay::ownRobotStore::getOwnRobot().getRole();

	    auto assistantRole = teamplay::teamMatesStore::getTeamMatesIncludingGoalie().getAssistantOfRole(myRole);

	    if (assistantRole)
	    {
	    	ballPossession_struct_t ball_possession;
			cWorldModelInterface::getInstance().getBallPossession(ball_possession);
	    	if (    (ball_possession.possessionType == ballPossessionEnum::TEAMMEMBER)
	    		 && (ball_possession.robotID == assistantRole->getNumber()) )
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

bool allRobotsActive()
{
	bool retVal = false;

	try
	{
	    auto NrOfTeammates = teamplay::teamMatesStore::getTeamMatesIncludingGoalie().getNumberOfRobots();

	    if (NrOfTeammates == 4)
	    {
	    		retVal = true;
	    }
	}
	catch (exception &e)
	{
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
	}

	return retVal;
}

bool defendingStrategyOn()
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

bool multipleOpponentsOnOwnHalf()
{
	bool retVal = false;

	try
	{
		int nrOfOpponentsOnOwnHalf = 0;

        robotLocations opponents;
        cWorldModelInterface::getInstance().getOpponents(opponents);

        for (auto opponent = opponents.begin(); opponent != opponents.end(); opponent++)
        {
            geometry::Pose2D opp_location = opponent->second.position;

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

bool isOpponentWithinXMeterFromOwnGoal()
{
	bool retVal = false;

	try
	{
		double dist_threshold_to_goal = 5.5;

		// Get closest opponent to own goal
        Point2D own_goalline_center = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OWN_GOALLINE_CENTER);
		float closestOppToOwnGoal_x;
		float closestOppToOwnGoal_y;
		bool closestOppToOwnGoalFound = cWorldStateFunctions::getInstance().getClosestOpponentToLocationXY(closestOppToOwnGoal_x, closestOppToOwnGoal_y, (float) own_goalline_center.x, (float) own_goalline_center.y);

		// check if closest opponent is within distance threshold
		if (closestOppToOwnGoalFound)
		{
			if (calc_distance((double) own_goalline_center.x,(double) own_goalline_center.y,(double) closestOppToOwnGoal_x,(double) closestOppToOwnGoal_y) < dist_threshold_to_goal)
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

bool cWorldStateFunctions::getPreferredShootXYOfGoal(float &target_x, float &target_y)
{
	bool retVal = true; // always return preferred part of goal, default middle of goal

	try
	{
	    // own position
	    Position2D myPos;
	    cWorldModelInterface::getInstance().getOwnLocation(myPos);

	    // some variables that read nice
	    auto goalCenter = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OPP_GOALLINE_CENTER);
	    auto goalPostLeft = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OPP_GOALPOST_LEFT);
	    auto goalPostRight = teamplay::fieldDimensionsStore::getFieldDimensions().getLocation(teamplay::fieldPOI::OPP_GOALPOST_RIGHT);

	    double goalPostShootOffset = 0.35; // the robot should shoot inside the goal area i.o. onto the goalpost
	    goalPostLeft.x = goalPostLeft.x + goalPostShootOffset;
	    goalPostRight.x = goalPostRight.x - goalPostShootOffset;

	    // default aim is opponent goalline center
	    auto preferredPartOfGoal = goalCenter;

	    // only take opp goalkeeper into account when close enough to goal (due to shot inaccuracy long distances)
		if ( isOpponentGoalKeeperInLeftCorner() )
		{
			// Opponent goalkeeper is positioned in left corner, shoot in right corner
			preferredPartOfGoal = goalPostRight;
		}
		else if ( isOpponentGoalKeeperInRightCorner() )
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

// worldstatefunctions::getPreferredShotLocationInGoal
// TODO: implement getPreferredShotLocationInGoal in
// TODO: Lobshot line_y > aiming center goal opschuiven
