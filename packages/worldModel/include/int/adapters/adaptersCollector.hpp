 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * adaptersCollector.hpp
 *
 *  Created on: Oct 6, 2016
 *      Author: Tim Kouters
 */

#ifndef ADAPTERSCOLLECTOR_HPP_
#define ADAPTERSCOLLECTOR_HPP_

#include <vector>
#include <map>

#include "int/types/ball/ballMeasurementType.hpp"
#include "int/types/ball/ballType.hpp"
#include "int/types/obstacle/obstacleMeasurementType.hpp"
#include "int/types/obstacle/obstacleType.hpp"
#include "int/types/robot/robotDisplacementType.hpp"
#include "int/types/robot/robotStatusType.hpp"
#include "int/types/robot/robotType.hpp"
#include "int/types/robot/robotMeasurementType.hpp"

#include "int/administrators/ballAdministrator.hpp"
#include "int/administrators/obstacleAdministrator.hpp"
#include "int/administrators/robotAdministrator.hpp"

#include "int/adapters/worldModelInfoUpdater.hpp"
#include "int/adapters/worldModelSyncInfoUpdater.hpp"

class adaptersCollector
{
	public:
		adaptersCollector();
		~adaptersCollector();

		void setBallAdministrator(ballAdministrator *ballAdmin);
		void setObstacleAdministrator(obstacleAdministrator *obstacleAdmin);
		void setRobotAdministrator(robotAdministrator *robotAdmin);
		void setWorldModelInfoUpdater(worldModelInfoUpdater *wmInfo);
		void setWorldModelSyncInfoUpdater(worldModelSyncInfoUpdater *wmSyncInfo);
		void updateVisionRobotMeasurement(const std::vector<robotMeasurementClass_t> measurement);
		void updateVisionBallMeasurement(const std::vector<ballMeasurementType> measurement);
		void updateVisionObstacleMeasurement(const std::vector<obstacleMeasurementType> measurement);
		void updateVisionBallPossession(const bool ballIsSeenInFrontOfBallHandlers);
		void updatePeripheralsDisplacement(const std::vector<robotDisplacementClass_t> displacements);
		void updatePeripheralsBallPossession(const bool ballIsCaughtByBallHandlers);
		void updatePeripheralsRobotStatus(const robotStatusType status);
		void updateWmSyncBallMeasurement(const std::vector<ballMeasurementType> measurements);
		void updateWmSyncObstacleMeasurement(const std::vector<obstacleMeasurementType> measurements);
		void updateWmSyncBallPossession(const std::map<uint8_t, bool> possession);
		void updateWmSyncTeamMember(const robotClass_t robot);
		void heartBeatRecalculation(bool dummy);

		/*
		 * Overruling functions
		 */
		void overruleRobotPosition(const robotClass_t robot);
		void overruleBallPosition(const ballClass_t ball);
		void overruleObstaclePostions(const std::vector<obstacleClass_t> obstacles);
        void overruleRobotStatus(const robotStatusType status);

	private:
		ballAdministrator *_ballAdmin;
		obstacleAdministrator *_obstacleAdmin;
		robotAdministrator *_robotAdmin;
		worldModelInfoUpdater *_wmInfo;
		worldModelSyncInfoUpdater *_wmSyncInfo;
		bool _ballIsCaughtByBallHandlers;
		robotStatusType _robotStatus;
};

#endif /* ADAPTERSCOLLECTOR_HPP_ */
