 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * robotAdministratorMock.hpp
 *
 *  Created on: Nov 25, 2016
 *      Author: Tim Kouters
 */

#ifndef ROBOTADMINISTRATORMOCK_HPP_
#define ROBOTADMINISTRATORMOCK_HPP_

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "int/administrators/robotAdministrator.hpp"

class robotAdministratorMock : public robotAdministrator
{
	public:
		MOCK_METHOD1(appendRobotVisionMeasurements, void(const std::vector<robotMeasurementClass_t> measurements));
		MOCK_METHOD1(appendRobotDisplacementMeasurements, void(const std::vector<robotDisplacementClass_t> displacements));
		MOCK_METHOD1(updateRobotPositionAndVelocity, void(const robotClass_t robot));
		MOCK_METHOD0(disableOverrulingOfLocalRobot, void());
		MOCK_METHOD2(claimBallPossession, void(const uint8_t robotID, const ballClaimType claimType));
		MOCK_METHOD2(releaseBallPossession, void(const uint8_t robotID, const ballClaimType releaseType));
		MOCK_METHOD2(setRobotStatus, void(const uint8_t robotID, const robotStatusType status));

		MOCK_METHOD1(performCalculation, void(const double timeNow));

		MOCK_METHOD0(getLocalRobotPosition, robotClass_t());
		MOCK_METHOD0(getTeammembers, std::vector<robotClass_t>());
		MOCK_METHOD0(getActiveMembers, std::vector<uint8_t>());
		MOCK_METHOD0(getLocalBallPossession, ballPossessionClass_t());
		MOCK_METHOD0(getBallPossession, ballPossessionClass_t());

	private:
		MOCK_METHOD1(removeTimedoutRobots, void(const double timeNow));
};

#endif /* ROBOTADMINISTRATORMOCK_HPP_ */
