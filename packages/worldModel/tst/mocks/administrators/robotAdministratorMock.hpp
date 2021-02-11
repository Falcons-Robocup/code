// Copyright 2016-2019 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
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

    	MOCK_METHOD1(performCalculation, void(rtime const timeNow));

    	MOCK_METHOD1(getLocalRobotPosition, robotClass_t(rtime const timeNow));
    	MOCK_METHOD0(getTeammembers, std::vector<robotClass_t>());
    	MOCK_METHOD0(getActiveMembers, std::vector<uint8_t>());
    	MOCK_METHOD0(getLocalBallPossession, ballPossessionClass_t());
    	MOCK_METHOD0(getBallPossession, ballPossessionClass_t());

    private:
    	MOCK_METHOD1(removeTimedoutRobots, void(rtime const timeNow));
};

#endif /* ROBOTADMINISTRATORMOCK_HPP_ */
