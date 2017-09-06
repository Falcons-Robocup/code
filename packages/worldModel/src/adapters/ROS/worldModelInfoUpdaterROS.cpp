 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * worldModelInfoUpdaterROS.cpp
 *
 *  Created on: Oct 8, 2016
 *      Author: Tim Kouters
 */

#include "int/adapters/ROS/worldModelInfoUpdaterROS.hpp"

#include "int/configurators/ballTrackerConfigurator.hpp"

#include "ext/WorldModelNames.h"
#include "int/facilities/ROSConvert.hpp"

#include "cDiagnosticsEvents.hpp"
#include "FalconsCommon.h"

worldModelInfoUpdaterROS::worldModelInfoUpdaterROS()
{
	_diagSender = new diagnostics::cDiagnosticsSender<rosMsgs::t_diag_wm_top>(diagnostics::DIAG_WM_TOP, 10, false);
}

worldModelInfoUpdaterROS::~worldModelInfoUpdaterROS()
/*
 * Chuck Norris tells Simon what to do
 */
{

}

void worldModelInfoUpdaterROS::InitializeROS()
{
	try
	{
		_hROS.reset(new ros::NodeHandle());
		_pWmInfo = _hROS->advertise<worldModel::t_wmInfo>(WorldModelInterface::t_wmInfo, 1, false);
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void worldModelInfoUpdaterROS::setBallLocation(ballClass_t ball)
{
	/*
	 * Add own ball if ball is valid
	 * If not, see if we have ball possession
	 * If not, see if teammember has ball possession
	 * Adjust ball location accordingly
	 */

	_msgWmInfo.ballX = ball.getX();
	_msgWmInfo.ballY = ball.getY();
	_msgWmInfo.ballZ = ball.getZ();
	_msgWmInfo.ballVX = ball.getVX();
	_msgWmInfo.ballVY = ball.getVY();
	_msgWmInfo.ballVZ = ball.getVZ();
	_msgWmInfo.ballConfidence = ball.getConfidence();
	_msgWmInfo.ballTimestamp = ball.getTimestamp();
	_msgWmInfo.isBallValid = ball.getIsValid();

	if(!ball.getIsValid())
	{
		if(_msgWmInfo.possession.type == rosMsgs::BallPossession::TYPE_TEAMMEMBER)
		{
			if(_msgWmInfo.possession.robotID == getRobotNumber())
			{
				_msgWmInfo.ballX = _msgWmInfo.locationX;
				_msgWmInfo.ballY = _msgWmInfo.locationY;
				_msgWmInfo.ballZ = 0.0;
				_msgWmInfo.ballVX = _msgWmInfo.locationVx;
				_msgWmInfo.ballVY = _msgWmInfo.locationVy;
				_msgWmInfo.ballVZ = 0.0;
				_msgWmInfo.ballConfidence = 1.0;
				_msgWmInfo.ballTimestamp = getTimeNow();
				_msgWmInfo.isBallValid = true;
			}
			else
			{
				for(size_t i = 0; (i < _msgWmInfo.nrOfTeamMembers); i++)
				{
					if(_msgWmInfo.teamMemberRobotID.at(i) == _msgWmInfo.possession.robotID)
					{
						_msgWmInfo.ballX = _msgWmInfo.teamMemberX.at(i);
						_msgWmInfo.ballY = _msgWmInfo.teamMemberY.at(i);
						_msgWmInfo.ballZ = 0.0;
						_msgWmInfo.ballVX = _msgWmInfo.teamMemberVx.at(i);
						_msgWmInfo.ballVY = _msgWmInfo.teamMemberVy.at(i);
						_msgWmInfo.ballVZ = 0.0;
						_msgWmInfo.ballConfidence = 1.0;
						_msgWmInfo.ballTimestamp = getTimeNow();
						_msgWmInfo.isBallValid = true;
					}
				}
			}
		}
	}
}

void worldModelInfoUpdaterROS::setBallPossession(ballPossessionClass_t possession)
{
	_msgWmInfo.possession = convertBallPossession(possession);
	possession.getClaimedPosition(_msgWmInfo.possessionX, _msgWmInfo.possessionY);

	_diagMsg.ballPossessionBallHandlers = possession.ballHandlersClaimedPossession();
	_diagMsg.ballPossessionVision = possession.visionClaimedPossession();
}

void worldModelInfoUpdaterROS::setObstacles(std::vector<obstacleClass_t> obstacles)
{
	_msgWmInfo.obstacleConfidence.clear();
	_msgWmInfo.obstacleTimestamp.clear();
	_msgWmInfo.obstacleX.clear();
	_msgWmInfo.obstacleY.clear();
	_msgWmInfo.obstacleVX.clear();
	_msgWmInfo.obstacleVY.clear();

	_msgWmInfo.nrObstacleMeasurements = obstacles.size();
	for(auto it = obstacles.begin(); it != obstacles.end(); it++)
	{
		_msgWmInfo.obstacleConfidence.push_back(it->getConfidence());
		_msgWmInfo.obstacleTimestamp.push_back(it->getTimestamp());
		_msgWmInfo.obstacleX.push_back(it->getX());
		_msgWmInfo.obstacleY.push_back(it->getY());
		_msgWmInfo.obstacleVX.push_back(it->getVX());
		_msgWmInfo.obstacleVY.push_back(it->getVY());
	}
}

void worldModelInfoUpdaterROS::setRobotLocation(robotClass_t robotLocation)
{
	_msgWmInfo.locationTimestamp = robotLocation.getTimestamp();
	_msgWmInfo.locationX = robotLocation.getX();
	_msgWmInfo.locationY = robotLocation.getY();
	_msgWmInfo.locationTheta = robotLocation.getTheta();
	_msgWmInfo.locationVx = robotLocation.getVX();
	_msgWmInfo.locationVy = robotLocation.getVY();
	_msgWmInfo.locationVtheta = robotLocation.getVTheta();
}

void worldModelInfoUpdaterROS::setActiveRobots(std::vector<uint8_t> activeRobots)
{
    _msgWmInfo.activeRobots.clear();
	for(auto it = activeRobots.begin(); it != activeRobots.end(); it++)
	{
		_msgWmInfo.activeRobots.push_back(*it);
	}

	_msgWmInfo.ownRobotIsActive = (std::find(activeRobots.begin(), activeRobots.end(), getRobotNumber()) != activeRobots.end());
}

void worldModelInfoUpdaterROS::setRobotStatus(robotStatusType status)
{
	_msgWmInfo.robotStatus = convertRobotStatus(status);
}

void worldModelInfoUpdaterROS::setTeamMembers(std::vector<robotClass_t> members)
{
	_msgWmInfo.teamMemberX.clear();
	_msgWmInfo.teamMemberY.clear();
	_msgWmInfo.teamMemberTheta.clear();
	_msgWmInfo.teamMemberVx.clear();
	_msgWmInfo.teamMemberVy.clear();
	_msgWmInfo.teamMemberVtheta.clear();
	_msgWmInfo.teamMemberTimestamp.clear();
	_msgWmInfo.teamMemberRobotID.clear();

	_msgWmInfo.nrOfTeamMembers = members.size();
	for(auto it = members.begin(); it != members.end(); it++)
	{
		_msgWmInfo.teamMemberX.push_back(it->getX());
		_msgWmInfo.teamMemberY.push_back(it->getY());
		_msgWmInfo.teamMemberTheta.push_back(it->getTheta());
		_msgWmInfo.teamMemberVx.push_back(it->getVX());
		_msgWmInfo.teamMemberVy.push_back(it->getVY());
		_msgWmInfo.teamMemberVtheta.push_back(it->getVTheta());
		_msgWmInfo.teamMemberTimestamp.push_back(it->getTimestamp());
		_msgWmInfo.teamMemberRobotID.push_back(it->getRobotID());
	}
}

void worldModelInfoUpdaterROS::sendPacket()
{
	try
	{
		// handle generic worldModel diagnostics (inplay, ball possession, etc)
		sendDiagnostics();

		_pWmInfo.publish(_msgWmInfo);
	}
	catch(std::exception &e)
	{
		TRACE_ERROR("Caught exception: %s", e.what());
		std::cout << "Caught exception: " << e.what() << std::endl;
		throw std::runtime_error(std::string("Linked to: ") + e.what());
	}
}

void worldModelInfoUpdaterROS::sendDiagnostics()
{
    if (_diagSender != NULL)
    {
        _diagMsg.ballPossession.type = _msgWmInfo.possession.type;
        _diagMsg.ballPossession.robotID = _msgWmInfo.possession.robotID;
        _diagMsg.ball.confidence = _msgWmInfo.ballConfidence;
        _diagMsg.ball.id = 1000 * getRobotNumber() + 50;
        _diagMsg.ball.x = _msgWmInfo.ballX;
        _diagMsg.ball.y = _msgWmInfo.ballY;
        _diagMsg.ball.z = _msgWmInfo.ballZ;
        _diagMsg.ball.vx = _msgWmInfo.ballVX;
        _diagMsg.ball.vy = _msgWmInfo.ballVY;
        _diagMsg.ball.vz = _msgWmInfo.ballVZ;

        _diagMsg.obstacles.clear();
        for(size_t i = 0; i < _msgWmInfo.nrObstacleMeasurements; i++)
        {
        	rosMsgs::t_obstacle obstacle;

        	obstacle.id = 1000 * getRobotNumber() + i;
        	obstacle.confidence = _msgWmInfo.obstacleConfidence.at(i);
        	obstacle.x = _msgWmInfo.obstacleX.at(i);
        	obstacle.y= _msgWmInfo.obstacleY.at(i);
        	obstacle.vx = _msgWmInfo.obstacleVX.at(i);
        	obstacle.vy  = _msgWmInfo.obstacleVY.at(i);

        	_diagMsg.obstacles.push_back(obstacle);
        }

        _diagMsg.inplay = _msgWmInfo.ownRobotIsActive;

        _diagMsg.teamActivity = "";
	    for(auto it = _msgWmInfo.activeRobots.begin(); it != _msgWmInfo.activeRobots.end(); it++)
	    {
		    _diagMsg.teamActivity += boost::lexical_cast<std::string>((int)(*it)) + " ";
	    }

        _diagSender->set(_diagMsg);
    }
    
    // generate event at state change for ballPossession and only for current robot
    static auto lastBallPossession = _msgWmInfo.possession;
    if (lastBallPossession.type != _msgWmInfo.possession.type)
    {
        if ((_msgWmInfo.possession.type == rosMsgs::BallPossession::TYPE_FIELD) && (lastBallPossession.type == rosMsgs::BallPossession::TYPE_TEAMMEMBER) && (lastBallPossession.robotID == getRobotNumber()))
        {
            TRACE_INFO("lost the ball");
        }
        if ((_msgWmInfo.possession.type == rosMsgs::BallPossession::TYPE_TEAMMEMBER) && (_msgWmInfo.possession.robotID == getRobotNumber()))
        {
            TRACE_INFO("got the ball at (%6.2f, %6.2f)", _msgWmInfo.locationX, _msgWmInfo.locationY);
        }
    }
    lastBallPossession = _msgWmInfo.possession;
}

