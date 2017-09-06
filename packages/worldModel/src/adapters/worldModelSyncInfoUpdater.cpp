 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * worldModelSyncInfoUpdater.cpp
 *
 *  Created on: Oct 9, 2016
 *      Author: Tim Kouters
 */

#include "int/adapters/worldModelSyncInfoUpdater.hpp"

#include "cDiagnosticsEvents.hpp"

worldModelSyncInfoUpdater::worldModelSyncInfoUpdater()
{
	_ballAdmin = NULL;
	_obstacleAdmin = NULL;
	_robotAdmin = NULL;
}

worldModelSyncInfoUpdater::~worldModelSyncInfoUpdater()
/*
 * Chuck Norris can pick oranges from an apple tree
 * and make the best lemonade you've ever tasted
 */
{

}


void worldModelSyncInfoUpdater::setBallAdministrator(ballAdministrator *ballAdmin)
{
	_ballAdmin = ballAdmin;
}

void worldModelSyncInfoUpdater::setObstacleAdministrator(obstacleAdministrator *obstacleAdmin)
{
	_obstacleAdmin = obstacleAdmin;
}

void worldModelSyncInfoUpdater::setRobotAdministrator(robotAdministrator *robotAdmin)
{
	_robotAdmin = robotAdmin;
}

void worldModelSyncInfoUpdater::notify()
{
	try
	{
		std::vector<ballMeasurementType> balls;
		ballPossessionClass_t ballPossession;
		std::vector<obstacleMeasurementType> obstacles;
		robotClass_t robotLocation;
		robotStatusType robotStatus = robotStatusType::OUTOFPLAY;

		if((_ballAdmin != NULL) &&
		   (_obstacleAdmin != NULL) &&
		   (_robotAdmin != NULL))
		{
			/*
			 * Only select the ball with the higher confidence
			 * This is the first ball in the vector
			 */
			_ballAdmin->getLocalBallMeasurements(balls);
			_fncSetBallLocations(balls);

			/*
			 * Fill the rest
			 */
			ballPossession = _robotAdmin->getLocalBallPossession();
			_fncSetBallPossession(ballPossession);

			_obstacleAdmin->getLocalObstacleMeasurements(obstacles);
			_fncSetObstacles(obstacles);

			robotLocation = _robotAdmin->getLocalRobotPosition();
			_fncSetRobotLocation(robotLocation);

			std::vector<uint8_t> activeMembers = _robotAdmin->getActiveMembers();
			if(find(activeMembers.begin(), activeMembers.end(), getRobotNumber()) != activeMembers.end())
			{
				robotStatus = robotStatusType::INPLAY;
			}
			_fncSetRobotStatus(robotStatus);

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
