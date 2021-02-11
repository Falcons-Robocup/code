// Copyright 2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * simworldTest.cpp
 *
 *  Created on: Feb 2, 2019
 *      Author: Coen Tempelaars
 */

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "int/gameDataFactory.hpp"
#include "int/simworld.hpp"

using testing::_;
using testing::Return;
using testing::Test;

MATCHER_P(BallPositionMatcher, expected, "")
{
    return arg.ball.getPosition() == expected;
}

MATCHER_P3(RobotVelocityMatcher, teamID, robotID, expected, "")
{
    return (   (abs(expected.x   - arg.team.at(teamID).at(robotID).getVelocity().x)   < 0.01)
            && (abs(expected.y   - arg.team.at(teamID).at(robotID).getVelocity().y)   < 0.01)
            && (abs(expected.phi - arg.team.at(teamID).at(robotID).getVelocity().phi) < 0.01));
}

class MockConfigAdapter : public AbstractConfigAdapter {
public:
    MOCK_CONST_METHOD0(getArbiter, std::string());
    MOCK_CONST_METHOD1(getSize, int(const TeamID));
};


class MockGameDataAdapter : public AbstractGameDataAdapter {
public:
    MOCK_CONST_METHOD1(publishGameData, void(const GameData&));
    MOCK_CONST_METHOD3(publishGameData, void(const GameData&, const TeamID, const RobotID));
    MOCK_CONST_METHOD1(publishScene, void(const GameData&));
    MOCK_METHOD1(checkUpdatedScene, bool(SimulationScene &));
};


class MockMotionAdapter : public AbstractMotionAdapter {
public:
    MOCK_CONST_METHOD2(getVelocity, Velocity2D(const TeamID&, const RobotID&));
    MOCK_CONST_METHOD2(getKickerData, Kicker(const TeamID&, const RobotID&));
    MOCK_CONST_METHOD2(hasBallHandlersEnabled, bool(const TeamID&, const RobotID&));
};


class TestableSimworld : public Simworld
{
public:
    TestableSimworld(AbstractConfigAdapter& configAdapter,
                     AbstractGameDataAdapter& gameDataAdapter,
                     AbstractMotionAdapter& motionAdapter)
    {
        _configAdapter = &configAdapter;
        _gameDataAdapter = &gameDataAdapter;
        _motionAdapter = &motionAdapter;
    }
};


class ASimworld : public Test
{
public:
    ASimworld()
    : simworld(mockConfigAdapter, mockGameDataAdapter, mockMotionAdapter)
    {
        ON_CALL(mockConfigAdapter, getArbiter()).WillByDefault(Return("user"));
        ON_CALL(mockConfigAdapter, getSize(_)).WillByDefault(Return(5));
        ON_CALL(mockMotionAdapter, getVelocity(_,_)).WillByDefault(Return(Velocity2D()));
        ON_CALL(mockMotionAdapter, getKickerData(_,_)).WillByDefault(Return(Kicker()));
        ON_CALL(mockMotionAdapter, hasBallHandlersEnabled(_,_)).WillByDefault(Return(false));

        simworld.initialize();
    }

    MockConfigAdapter mockConfigAdapter;
    MockGameDataAdapter mockGameDataAdapter;
    MockMotionAdapter mockMotionAdapter;
    GameData gameData;
    TestableSimworld simworld;
};


TEST_F(ASimworld, ControlsTheWorld)
{
    EXPECT_CALL(mockMotionAdapter, getVelocity(_,_)).Times(10);
    EXPECT_CALL(mockMotionAdapter, getKickerData(_,_)).Times(10);
    EXPECT_CALL(mockMotionAdapter, hasBallHandlersEnabled(_,_)).Times(10);

    EXPECT_CALL(mockGameDataAdapter, publishGameData(BallPositionMatcher(Point3D(0.0, 0.0, 0.0))));
    EXPECT_CALL(mockGameDataAdapter, publishGameData(_,_,_)).Times(10);

    simworld.control();
}


TEST_F(ASimworld, PublishesVelocityForTeamA)
{
    EXPECT_CALL(mockMotionAdapter, getVelocity(_,_)).Times(10).WillRepeatedly(Return(Velocity2D(0.10, -0.06, -0.12)));
    EXPECT_CALL(mockMotionAdapter, getKickerData(_,_)).Times(10);
    EXPECT_CALL(mockMotionAdapter, hasBallHandlersEnabled(_,_)).Times(10);

    EXPECT_CALL(mockGameDataAdapter, publishGameData(RobotVelocityMatcher(TeamID::A, RobotID::r5, Velocity2D(0.06, 0.10, -0.12))));
    EXPECT_CALL(mockGameDataAdapter, publishGameData(_,_,_)).Times(10);

    simworld.control();
}


TEST_F(ASimworld, PublishesVelocityForTeamB)
{
    EXPECT_CALL(mockMotionAdapter, getVelocity(_,_)).Times(10).WillRepeatedly(Return(Velocity2D(0.10, -0.06, -0.12)));
    EXPECT_CALL(mockMotionAdapter, getKickerData(_,_)).Times(10);
    EXPECT_CALL(mockMotionAdapter, hasBallHandlersEnabled(_,_)).Times(10);

    EXPECT_CALL(mockGameDataAdapter, publishGameData(RobotVelocityMatcher(TeamID::B, RobotID::r5, Velocity2D(0.06, 0.10, -0.12))));
    EXPECT_CALL(mockGameDataAdapter, publishGameData(_,_,_)).Times(10);

    simworld.control();
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
