 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * testCollectorAdapter.cpp
 *
 *  Created on: Nov 25, 2016
 *      Author: Tim Kouters
 */

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "int/adapters/adaptersCollector.hpp"

#include "../mocks/administrators/ballAdministratorMock.hpp"
#include "../mocks/administrators/obstacleAdministratorMock.hpp"
#include "../mocks/administrators/robotAdministratorMock.hpp"
#include "../mocks/adapters/worldModelInfoUpdaterMock.hpp"
#include "../mocks/adapters/worldModelSyncInfoUpdaterMock.hpp"

class collectorAdapterTester : public ::testing::Test
{
	protected:
		/* SUT */
		adaptersCollector _adpCollector;

		/* Mocks */
		robotAdministratorMock _admRobot;
		ballAdministratorMock _admBall;
		obstacleAdministratorMock _admObstacle;
		worldModelInfoUpdaterMock _wmInfo;
		worldModelSyncInfoUpdaterMock _wmSyncInfo;

		virtual void SetUp()
		{
			_adpCollector.setBallAdministrator(&_admBall);
			_adpCollector.setObstacleAdministrator(&_admObstacle);
			_adpCollector.setRobotAdministrator(&_admRobot);
			_adpCollector.setWorldModelInfoUpdater(&_wmInfo);
			_adpCollector.setWorldModelSyncInfoUpdater(&_wmSyncInfo);
		}

		virtual void TearDown()
		{

		}
};

TEST_F(collectorAdapterTester, addingVisionMeasurement)
{
	/*
	 * Setup
	 */

	//EXPECT_CALL(_admRobot, appendRobotVisionMeasurements(::testing::_)).
	//		Times(1);

	/*
	 * Execution
	 */
	std::vector<robotMeasurementClass_t> measurements;

	robotMeasurementClass_t measurement;
	measurements.push_back(measurement);

	//_adpCollector.updateVisionRobotMeasurement(measurements);


	/*
	 * Verification
	 */
}

TEST_F(collectorAdapterTester, addingBallMeasurement)
{
	/*
	 * Setup
	 */
//	EXPECT_CALL(_admBall, appendBallMeasurements(::testing::_)).
//			Times(::testing:: Exactly(1));
//	EXPECT_CALL(_admRobot, getLocalRobotPosition()).WillOnce(::testing::Return(const robotClass_t));

	/*
	 * Execution
	 */
	std::vector<ballMeasurementType> measurements;

	ballMeasurementType measurement;
	measurements.push_back(measurement);

	//_adpCollector.updateVisionBallMeasurement(measurements);


	/*
	 * Verification
	 */
}

TEST_F(collectorAdapterTester, verifyCalculationsAndNotifications)
{
	/*
	 * Setup
	 */
	EXPECT_CALL(_admBall, performCalculation(::testing::A<double>())).
			Times(1);
	EXPECT_CALL(_admObstacle, performCalculation(::testing::A<double>())).
			Times(1);
	EXPECT_CALL(_admRobot, performCalculation(::testing::A<double>())).
			Times(1);
	EXPECT_CALL(_wmInfo, notify()).
			Times(1);
	EXPECT_CALL(_wmSyncInfo, notify()).
			Times(1);

	/*
	 * Execution
	 */
	_adpCollector.heartBeatRecalculation(true);


	/*
	 * Verification
	 */
}
TEST_F(collectorAdapterTester, verifyRobotAdminNullPointer)
{
	/*
	 * Setup
	 */
	_adpCollector.setRobotAdministrator(NULL);


	EXPECT_CALL(_admBall, performCalculation(::testing::A<double>())).
			Times(::testing::Exactly(0));
	EXPECT_CALL(_admObstacle, performCalculation(::testing::A<double>())).
			Times(::testing::Exactly(0));
	EXPECT_CALL(_admRobot, performCalculation(::testing::A<double>())).
			Times(::testing::Exactly(0));
	EXPECT_CALL(_wmInfo, notify()).
			Times(::testing::Exactly(0));
	EXPECT_CALL(_wmSyncInfo, notify()).
			Times(::testing::Exactly(0));
	/*
	 * Execution
	 */
	EXPECT_NO_THROW(_adpCollector.heartBeatRecalculation(true));


	/*
	 * Verification
	 */
}

TEST_F(collectorAdapterTester, verifyBallAdminNullPointer)
{
	/*
	 * Setup
	 */
	_adpCollector.setBallAdministrator(NULL);


	EXPECT_CALL(_admBall, performCalculation(::testing::A<double>())).
			Times(::testing::Exactly(0));
	EXPECT_CALL(_admObstacle, performCalculation(::testing::A<double>())).
			Times(::testing::Exactly(0));
	EXPECT_CALL(_admRobot, performCalculation(::testing::A<double>())).
			Times(::testing::Exactly(0));
	EXPECT_CALL(_wmInfo, notify()).
			Times(::testing::Exactly(0));
	EXPECT_CALL(_wmSyncInfo, notify()).
			Times(::testing::Exactly(0));

	/*
	 * Execution
	 */
	EXPECT_NO_THROW(_adpCollector.heartBeatRecalculation(true));


	/*
	 * Verification
	 */
}

TEST_F(collectorAdapterTester, verifyObstacleAdminNullPointer)
{
	/*
	 * Setup
	 */
	_adpCollector.setObstacleAdministrator(NULL);


	EXPECT_CALL(_admBall, performCalculation(::testing::A<double>())).
			Times(::testing::Exactly(0));
	EXPECT_CALL(_admObstacle, performCalculation(::testing::A<double>())).
			Times(::testing::Exactly(0));
	EXPECT_CALL(_admRobot, performCalculation(::testing::A<double>())).
			Times(::testing::Exactly(0));
	EXPECT_CALL(_wmInfo, notify()).
			Times(::testing::Exactly(0));
	EXPECT_CALL(_wmSyncInfo, notify()).
			Times(::testing::Exactly(0));

	/*
	 * Execution
	 */
	EXPECT_NO_THROW(_adpCollector.heartBeatRecalculation(true));


	/*
	 * Verification
	 */
}

TEST_F(collectorAdapterTester, verifyWorldModelInfoNullPointer)
{
	/*
	 * Setup
	 */
	_adpCollector.setWorldModelInfoUpdater(NULL);


	EXPECT_CALL(_admBall, performCalculation(::testing::A<double>())).
			Times(::testing::Exactly(0));
	EXPECT_CALL(_admObstacle, performCalculation(::testing::A<double>())).
			Times(::testing::Exactly(0));
	EXPECT_CALL(_admRobot, performCalculation(::testing::A<double>())).
			Times(::testing::Exactly(0));
	EXPECT_CALL(_wmInfo, notify()).
			Times(::testing::Exactly(0));
	EXPECT_CALL(_wmSyncInfo, notify()).
			Times(::testing::Exactly(0));

	/*
	 * Execution
	 */
	EXPECT_NO_THROW(_adpCollector.heartBeatRecalculation(true));


	/*
	 * Verification
	 */
}

TEST_F(collectorAdapterTester, verifyWorldModelSyncInfoNullPointer)
{
	/*
	 * Setup
	 */
	_adpCollector.setWorldModelSyncInfoUpdater(NULL);


	EXPECT_CALL(_admBall, performCalculation(::testing::A<double>())).
			Times(::testing::Exactly(0));
	EXPECT_CALL(_admObstacle, performCalculation(::testing::A<double>())).
			Times(::testing::Exactly(0));
	EXPECT_CALL(_admRobot, performCalculation(::testing::A<double>())).
			Times(::testing::Exactly(0));
	EXPECT_CALL(_wmInfo, notify()).
			Times(::testing::Exactly(0));
	EXPECT_CALL(_wmSyncInfo, notify()).
			Times(::testing::Exactly(0));

	/*
	 * Execution
	 */
	EXPECT_NO_THROW(_adpCollector.heartBeatRecalculation(true));


	/*
	 * Verification
	 */
}

// MAIN
int main(int argc, char **argv)
{
    // Run all the tests that were declared with TEST()
    testing::InitGoogleTest(&argc, argv);
    ::testing::InitGoogleMock(&argc, argv);

    return RUN_ALL_TESTS();
}
