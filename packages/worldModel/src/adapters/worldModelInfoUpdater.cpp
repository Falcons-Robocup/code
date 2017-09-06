 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * worldModelInfoUpdater.cpp
 *
 *  Created on: Oct 8, 2016
 *      Author: Tim Kouters
 */

#include "int/adapters/worldModelInfoUpdater.hpp"

#include "int/configurators/obstacleTrackerConfigurator.hpp"
#include "int/configurators/ballTrackerConfigurator.hpp"

#include "cDiagnosticsEvents.hpp"

worldModelInfoUpdater::worldModelInfoUpdater()
{
	_ballAdmin = NULL;
	_obstacleAdmin = NULL;
	_robotAdmin = NULL;
}

worldModelInfoUpdater::~worldModelInfoUpdater()
/*
 * Chuck Norris can pick oranges from an apple tree
 * and make the best lemonade you've ever tasted
 */
{

}


void worldModelInfoUpdater::setBallAdministrator(ballAdministrator *ballAdmin)
{
	_ballAdmin = ballAdmin;
}

void worldModelInfoUpdater::setObstacleAdministrator(obstacleAdministrator *obstacleAdmin)
{
	_obstacleAdmin = obstacleAdmin;
}

void worldModelInfoUpdater::setRobotAdministrator(robotAdministrator *robotAdmin)
{
	_robotAdmin = robotAdmin;
}

void worldModelInfoUpdater::notify()
{
	try
	{
		ballClass_t ball;
		ballPossessionClass_t ballPossession;
		std::vector<obstacleClass_t> obstacles;
		robotClass_t robotLocation;
		robotStatusType robotStatus = robotStatusType::OUTOFPLAY;
		std::vector<robotClass_t> teamMembers;
		Area2D playField(-_xMargin, -_yMargin, _xMargin, _yMargin);

		if((_ballAdmin != NULL) &&
		   (_obstacleAdmin != NULL) &&
		   (_robotAdmin != NULL))
		{
			/*
			 * Only select the ball with the higher confidence
			 * This is the first ball in the vector
			 * Furthermore, give own balls with a radius of x meters prio over global balls
			 */
			std::vector<ballClass_t> ownBalls;
			std::vector<ballClass_t> balls;
			robotClass_t robot = _robotAdmin->getLocalRobotPosition();
			Vector2D robotLoc(robot.getX(), robot.getY());
			bool useOwnBalls = false;

			_ballAdmin->getOwnBalls(ownBalls);

			for(size_t i = 0; i < ownBalls.size(); i++)
			{
				Vector2D ballLocation(ownBalls.at(i).getX(), ownBalls.at(i).getY());

				if((robotLoc - ballLocation).size() < ballTrackerConfigurator::getInstance().get(ballTrackerConfiguratorFloats::friendlyMeasurementsDistance))
				{
					balls.push_back(ownBalls.at(i));
					useOwnBalls = true;
				}
			}

			if(!useOwnBalls)
			{
				_ballAdmin->getGlobalBalls(balls);
			}
		    else
			{
			    TRACE("fallback to local ball");
			}

			bool ballInserted = false;
			for(size_t i = 0; ((i < balls.size()) && (!ballInserted)); i++)
			{
				Position2D ballLocation(balls.at(i).getX(), balls.at(i).getY(), 0.0);
				if(playField.includesPosition(ballLocation))
				{
					ball = balls.at(i);
					ballInserted = true;
				}
				else
				{
					TRACE("Ignoring ball at location x:%6.2f, y:%6.2f", balls.at(i).getX(), balls.at(i).getY());
				}
			}

			_fncSetBallLocation(ball);

			/*
			 * Fill the rest
			 */
			robotLocation = _robotAdmin->getLocalRobotPosition();
			_fncSetRobotLocation(robotLocation);

			std::vector<uint8_t> activeMembers = _robotAdmin->getActiveMembers();
			if(find(activeMembers.begin(), activeMembers.end(), getRobotNumber()) != activeMembers.end())
			{
				robotStatus = robotStatusType::INPLAY;
			}
			_fncSetRobotStatus(robotStatus);
			_fncSetActiveRobots(activeMembers);

			teamMembers = _robotAdmin->getTeammembers();
			_fncSetTeamMembers(teamMembers);

			/*
			 * Filter own location and team members from obstacles before sending obstacles
			 *
			 * Teammember filtering is actually configurable: 
			 * default, worldModel does not report friendly robots as obstacle 
			 * because pathPlanning & teamplay would go crazy trying to avoid / counter its own shadow
			 * but for obstacle tracking testing & tuning, it is very useful to compare observed (vision) obstacle 
			 * with our own localization results, which is much more precise
			 * so in robotCLI.py we overrule the yaml
			 *
			 * Another important reason to send down the list of teammembers to obstacleAdministrator is 
			 * to make diagnostics and visualization consistent with actual SW behavior
			 */
			_obstacleAdmin->notifyOwnLocation(robotLocation);
			_obstacleAdmin->notifyTeamMembers(teamMembers);
			_obstacleAdmin->getObstacles(obstacles);
            
			_fncSetObstacles(obstacles);

            /*
             * Final part of ball possession calculation:
             * if ballPossession is FIELD and an obstacle is close by the ball, 
             * then set ballPossession to OPPONENT
             */
            ballPossession = _robotAdmin->getBallPossession();
            if (!ballPossession.hasBallPossession())
            {
                ballPossession.setBallCloseToObstacle(checkBallCloseToObstacle(ball, obstacles));
            }
            _fncSetBallPossession(ballPossession);

			/*
			 * Call the binded function to send the package
			 */
			_fncSendPacket();
		}
		else
		{
			TRACE_ERROR("One of the administrator pointers is NULL");
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

bool worldModelInfoUpdater::checkBallCloseToObstacle(ballClass_t const &ball, std::vector<obstacleClass_t> const &obstacles)
{
    Point2D ballPos = Point2D(ball.getX(), ball.getY());
    float proximityThreshold = 0.6; // TODO make configurable?
    for (auto itObst = obstacles.begin(); itObst != obstacles.end(); ++itObst)
    {
        Point2D obstaclePos = Point2D(itObst->getX(), itObst->getY());
        if (vectorsize(ballPos - obstaclePos) < proximityThreshold)
        {
            return true;
        }
    }
    return false;
}

void worldModelInfoUpdater::filterOutTeamMembers(const std::vector<robotClass_t> members, std::vector<obstacleClass_t> &obstacles)
{
	try
	{
		float xyTolerance = obstacleTrackerConfigurator::getInstance().get(obstacleTrackerConfiguratorFloats::trackerXYTolerance);

		for(auto itObst = obstacles.begin(); itObst != obstacles.end(); )
		{
			bool found = false;
			Point2D obstaclePos = Point2D(itObst->getX(), itObst->getY());

			for(auto itMember = members.begin(); ((itMember != members.end()) && (!found)); itMember++)
			{
				Point2D memberPos = Point2D(itMember->getX(), itMember->getY());
				found = (vectorsize(obstaclePos - memberPos) < xyTolerance);
			}

			if(found)
			{
				itObst = obstacles.erase(itObst);
			}
			else
			{
				itObst++;
			}
		}
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}
