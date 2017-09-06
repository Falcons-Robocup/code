 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cWorldModelInterface.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: Ivo Matthijssen
 */
#include <stdexcept>

#include "FalconsCommon.h"

#include "int/rules/ruleStimulatePassing.hpp"
#include "int/stores/ballStore.hpp"
#include "int/stores/fieldDimensionsStore.hpp"
#include "int/stores/ownRobotStore.hpp"
#include "int/stores/teamMatesStore.hpp"
#include "int/utilities/trace.hpp"
#include "int/cWorldModelInterface.hpp"

using namespace teamplay;

cWorldModelInterface::cWorldModelInterface()
{
	_ownRobotPositionValid=false;
	_ownRobotVelocityValid = false;
}

cWorldModelInterface::~cWorldModelInterface()
{

}

// getters are used all over TeamPlay
// setters are called by cRosAdapterWorldModel

void cWorldModelInterface::setOwnRobot(const Position2D& pos, const Velocity2D& vel)
{
    try
    {
        teamplay::ownRobotStore::getOwnRobot().setNumber(getRobotNumber());
        teamplay::ownRobotStore::getOwnRobot().setPosition(pos);
        teamplay::ownRobotStore::getOwnRobot().setVelocity(vel);
    }
    catch (std::exception& e)
    {
        TRACE_ERROR("Caught exception while adding own robot to the team: ") << e.what();
        throw std::runtime_error(std::string("Caught exception while adding own robot to the team: ") + e.what());
    }
}

bool cWorldModelInterface::getOwnLocation(Position2D &robotPosition)
{
	robotPosition = _ownRobotPosition;
	return _ownRobotPositionValid;
}

void cWorldModelInterface::setOwnLocation(Position2D const &robotPosition)
{
	_ownRobotPosition = robotPosition;
	if( isnan(_ownRobotPosition.x) || isnan(_ownRobotPosition.y) )
	{
		_ownRobotPositionValid=false;
	}
	else if ( fieldDimensionsStore::getFieldDimensions().isPositionInSafetyBoundaries(_ownRobotPosition.x, _ownRobotPosition.y) )
	{
		_ownRobotPositionValid=true;
	}
	else
	{
		_ownRobotPositionValid=false;
	}

}

bool cWorldModelInterface::getOwnVelocity(Velocity2D &robotVelocity)
{
    robotVelocity = _ownRobotVelocity;
    return _ownRobotVelocityValid;
}

void cWorldModelInterface::setOwnVelocity(Velocity2D const &robotVelocity)
{
    _ownRobotVelocity = robotVelocity;
    if( isnan(_ownRobotVelocity.x) || isnan(_ownRobotVelocity.y) )
    {
        _ownRobotVelocityValid=false;
    }
    else
    {
        _ownRobotVelocityValid=true;
    }

}

void cWorldModelInterface::getTeammembers(robotLocations &teammembers)
{
	teammembers = _robotTeammembers;
}

void cWorldModelInterface::setTeammembers(robotLocations const &teammembers, std::vector<robotNumber> const &activeRobots)
{
    /* New style: fill the store */
    try
    {
        for (auto member = teammembers.begin(); member != teammembers.end(); member++)
        {
            const int robot_id(member->first);
            for (auto active_robot_id = activeRobots.begin(); active_robot_id != activeRobots.end(); active_robot_id++)
            {
                if (robot_id == *active_robot_id)
                {
                    const Position2D pos(member->second.position.x, member->second.position.y, member->second.position.getPhi());
                    const Velocity2D vel(member->second.velocity.x, member->second.velocity.y, member->second.velocity.getPhi());
                    const robot robot(robot_id, treeEnum::R_ROBOT_STOP, pos, vel);
                    teamplay::teamMatesStore::getTeamMatesIncludingGoalie().add(robot);
                }
            }
        }
    }
    catch (std::exception& e)
    {
        TRACE_ERROR("Caught exception while teammembers to the team: ") << e.what();
        throw std::runtime_error(std::string("Caught exception while adding teammembers to the team: ") + e.what());
    }

    /* Old style: update member variables */
    _robotTeammembers = teammembers;
    _activeRobots = activeRobots;
}

void cWorldModelInterface::getOpponents(robotLocations &opponents)
{
	opponents = _robotObstacles;
}

void cWorldModelInterface::setOpponents(robotLocations const &opponents)
{
	_robotObstacles = opponents;
}

void cWorldModelInterface::setBallLocation(ballLocations const &balls, ballLocation const &lastKnownBallLocation)
{
    if (balls.empty())
    {
        ballStore::getBall().setPositionUnknown(lastKnownBallLocation.position);
        ballStore::getBall().setVelocityUnknown();
    }
    else
    {
        ballStore::getBall().setPosition(balls.at(0).position);
        ballStore::getBall().setVelocity(balls.at(0).velocity);
    }
}

void cWorldModelInterface::getBallPossession(ballPossession_struct_t &ballPossession)
{
    ballPossession = _ballPossession;
}

void cWorldModelInterface::setBallPossession(ballPossession_struct_t const &ballPossession)
{
    _ballPossession = ballPossession;

    if (ballPossession.possessionType == ballPossessionEnum::TEAMMEMBER)
    {
        ruleStimulatePassing::getInstance().robotClaimsBall(ballPossession.robotID);
    }
}

void cWorldModelInterface::setBallClaimedLocation(Point3D const &claimLocation)
{
    ballStore::getBall().setPositionClaimed(claimLocation);
}

void cWorldModelInterface::getActiveRobots(std::vector<robotNumber> &activeRobots)
{
	activeRobots = _activeRobots;
}
