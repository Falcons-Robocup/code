// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * velocityControlTest.cpp
 *
 *  Created on: July 2019
 *      Author: Jan Feitsma
 */


// Include testframework
#include "gtest/gtest.h"
#include "gmock/gmock.h"
using namespace ::testing;

#include <memory>
#include <boost/algorithm/string/replace.hpp>
#include "velocityControlTestDefaults.hpp"
#include "tracing.hpp"

#define NUMERICAL_TOLERANCE 1e-4


TEST(velocityControlTest, onTarget)
{
    TRACE_FUNCTION("");

    // Arrange
    auto vc = defaultVelocityControlSetup();

    // Act
    vc.calculate();

    // Assert
    auto vel = vc.data.resultVelocityRcs;
    EXPECT_EQ(vel.x, 0.0);
    EXPECT_EQ(vel.y, 0.0);
    EXPECT_EQ(vel.phi, 0.0);
}

TEST(velocityControlTest, stop)
{
    TRACE_FUNCTION("");

    // Arrange
    auto vc = defaultVelocityControlSetup();
    vc.data.target.pos = pose(1, 0, 0);
    vc.data.target.vel = pose(0, 0, 0);
    vc.data.robotPosVelMoveType = robotPosVelEnum::VEL_ONLY;

    // Act
    vc.calculate();

    // Assert
    auto vel = vc.data.resultVelocityRcs;
    EXPECT_EQ(vel.x, 0.0);
    EXPECT_EQ(vel.y, 0.0);
    EXPECT_EQ(vel.phi, 0.0);
}

TEST(velocityControlTest, simpleMove_shouldMoveForward_POSVEL)
{
    TRACE_FUNCTION("");

    // Arrange
    auto vc = defaultVelocityControlSetup();
    vc.data.target.pos = pose(1, 0, 0);
    vc.data.target.vel = pose(0, 0, 0);
    vc.data.robotPosVelMoveType = robotPosVelEnum::POSVEL;

    // Act
    vc.calculate();

    // Assert
    auto vel = vc.data.resultVelocityRcs;
    EXPECT_GT(vel.y, 1.0);
    EXPECT_LT(fabs(vel.x), 0.01);
    EXPECT_LT(fabs(vel.phi), 0.01);
}

TEST(velocityControlTest, simpleMove_shouldMoveForward_POS_ONLY)
{
    TRACE_FUNCTION("");

    // Arrange
    auto vc = defaultVelocityControlSetup();
    vc.data.target.pos = pose(1, 0, 0);
    vc.data.robotPosVelMoveType = robotPosVelEnum::POS_ONLY;

    // Act
    vc.calculate();

    // Assert
    auto vel = vc.data.resultVelocityRcs;
    EXPECT_GT(vel.y, 1.0);
    EXPECT_LT(fabs(vel.x), 0.01);
    EXPECT_LT(fabs(vel.phi), 0.01);
}

TEST(velocityControlTest, simpleMove_shouldMoveForward_VEL_ONLY)
{
    TRACE_FUNCTION("");

    // Arrange
    auto vc = defaultVelocityControlSetup();
    vc.data.target.pos = pose(2, 0, 0);
    vc.data.target.vel = pose(0, 2, 0);
    vc.data.robotPosVelMoveType = robotPosVelEnum::VEL_ONLY;

    // Act
    vc.calculate();

    // Assert
    auto vel = vc.data.resultVelocityRcs;
    EXPECT_GT(vel.y, 1.0);
    EXPECT_LT(fabs(vel.x), 0.01);
    EXPECT_LT(fabs(vel.phi), 0.01);
}

TEST(velocityControlTest, acceleration_watchdog)
{
    TRACE_FUNCTION("");

    // Arrange
    auto vc = defaultVelocityControlSetup();
    vc.data.currentMotionTypeConfig.limits.maxAccYforward = 4.0;
    vc.data.robot.velocity = pose(1, 1, 0);
    vc.data.target.pos = pose(1, 0, 0);

    // First iteration: acceleration limiter
    vc.data.timestamp.fromDouble(0.0);
    vc.calculate();
    auto vel = vc.data.resultVelocityRcs;
    EXPECT_NEAR(vel.y, 0.2, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.x, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.phi, 0.0, NUMERICAL_TOLERANCE);

    // Second iteration: acceleration limiter
    // (previous velocity setpoint and timestamp are remembered)
    vc.data.timestamp.fromDouble(vc.data.dt);
    vc.calculate();
    vel = vc.data.resultVelocityRcs;
    EXPECT_NEAR(vel.y, 0.4, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.x, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.phi, 0.0, NUMERICAL_TOLERANCE);

    // Simulate a temporary 'dry spill' in setpoint
    // (according to execution architecture, during such a period (typically STOP) vc is not called)
    vc.data.timestamp.fromDouble(100 * vc.data.dt);

    // Third iteration: check that acceleration is limited as in first iteration
    vc.calculate();
    vel = vc.data.resultVelocityRcs;
    EXPECT_NEAR(vel.y, 0.2, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.x, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.phi, 0.0, NUMERICAL_TOLERANCE);

}

// Rotation tests to verify robot always chooses the short rotation direction
// First parameter: robot orientation
// Second parameter: rotation direction (clockwise)
// 4 orientations (north/east/south/west) times two directions = 8 test cases
class RotationTest : public testing::TestWithParam<std::tuple<float, bool>>
{
public:
    virtual void SetUp(){}
    virtual void TearDown(){}
};

INSTANTIATE_TEST_CASE_P(velocityControlTest,
                        RotationTest,
                        ::testing::Values(
                            std::make_tuple(0.00, true),
                            std::make_tuple(0.00, false),
                            std::make_tuple(1.57, true),
                            std::make_tuple(1.57, false),
                            std::make_tuple(3.14, true),
                            std::make_tuple(3.14, false),
                            std::make_tuple(4.72, true),
                            std::make_tuple(4.72, false)
                            )
                        );

TEST_P(RotationTest, RotationTests)
{
    const ::testing::TestInfo* const test_info = ::testing::UnitTest::GetInstance()->current_test_info();
    TRACE_FUNCTION(test_info->name());

    // Parameters
    float Rz = std::get<0>(GetParam());
    bool left = std::get<1>(GetParam());
    TRACE("test parameters: Rz=%6.2f left=%d", Rz, (int)left);

    // Arrange
    auto vc = defaultVelocityControlSetup();
    vc.data.robot.position.Rz = Rz;
    vc.data.target.pos.Rz = Rz + (left ? 1.57 : -1.57);

    // Act
    vc.calculate();

    // Assert
    auto vel = vc.data.resultVelocityRcs;
    EXPECT_LT(fabs(vel.x), 0.01);
    EXPECT_LT(fabs(vel.y), 0.01);
    if (left)
    {
        EXPECT_GT(vel.phi, 0.5);
    }
    else
    {
        EXPECT_LT(vel.phi, -0.5);
    }
}

TEST(velocityControlTest, closebyXYnotRz_shouldContinue)
{
    TRACE_FUNCTION("");

    // Arrange
    auto vc = defaultVelocityControlSetup();
    vc.data.target.pos.Rz = 1.0;

    // Act
    vc.calculate();

    // Assert
    auto vel = vc.data.resultVelocityRcs;
    EXPECT_NEAR(vel.x, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.y, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_GT(vel.phi, 0.0);
}

TEST(velocityControlTest, closebyRznotXY_shouldContinue)
{
    TRACE_FUNCTION("");

    // Arrange
    auto vc = defaultVelocityControlSetup();
    vc.data.target.pos.y = 1.0;

    // Act
    vc.calculate();

    // Assert
    auto vel = vc.data.resultVelocityRcs;
    // moving towards y=1 FCS, so that's moving sideways (x in RCS)
    EXPECT_LT(vel.x, 0.0);
    EXPECT_NEAR(vel.y, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.phi, 0.0, NUMERICAL_TOLERANCE);
}

// Limiter tests to verify acceleration and deceleration limiters are properly avclied
// First parameter:  degree of freedom (X, Y or Rz)
// Second parameter: current velocity
// Third parameter:  new position setpoint (w.r.t. current = 0)
// Fourth parameter: expected velocity
class LimiterTest : public testing::TestWithParam<std::tuple<std::string, float, float, float>>
{
public:
    virtual void SetUp(){}
    virtual void TearDown(){}
};

INSTANTIATE_TEST_CASE_P(velocityControlTest,
                        LimiterTest,
                        ::testing::Values(
                            std::make_tuple("X",   1.0,  1.0,  1.1), // acc
                            std::make_tuple("X",   1.0, -0.2,  0.8), // dec
                            std::make_tuple("X",  -1.0,  0.2, -0.8), // dec
                            std::make_tuple("X",  -1.0, -1.0, -1.1), // acc
                            std::make_tuple("Y",   1.0,  1.0,  1.1), // acc
                            std::make_tuple("Y",   1.0, -0.2,  0.8), // dec
                            std::make_tuple("Y",  -1.0,  0.2, -0.8), // dec
                            std::make_tuple("Y",  -1.0, -1.0, -1.1), // acc
                            std::make_tuple("Rz",  1.0,  1.0,  1.1), // acc
                            std::make_tuple("Rz",  1.0, -0.2,  0.8), // dec
                            std::make_tuple("Rz", -1.0,  0.2, -0.8), // dec
                            std::make_tuple("Rz", -1.0, -1.0, -1.1)  // acc
                            )
                        );

TEST_P(LimiterTest, LimiterTests)
{
    const ::testing::TestInfo* const test_info = ::testing::UnitTest::GetInstance()->current_test_info();
    TRACE_FUNCTION(test_info->name());

    // Parameters
    std::string dof = std::get<0>(GetParam());
    float currentSpeed = std::get<1>(GetParam());
    float newPosition = std::get<2>(GetParam());
    float expectedSpeed = std::get<3>(GetParam());
    TRACE("test parameters: dof=%s currentSpeed=%6.2f newPosition=%6.2f expectedSpeed=%6.2f", dof.c_str(), currentSpeed, newPosition, expectedSpeed);

    // Arrange
    auto vc = defaultVelocityControlSetup();
    vc.data.currentMotionTypeConfig.velocityControllers.shortStroke.type = VelocitySetpointControllerTypeEnum::LINEAR;
    vc.data.currentMotionTypeConfig.limits.maxAccX = 2.0;
    vc.data.currentMotionTypeConfig.limits.maxAccYforward = 2.0;
    vc.data.currentMotionTypeConfig.limits.maxAccYbackward = 2.0;
    vc.data.currentMotionTypeConfig.limits.maxAccRz = 2.0;
    vc.data.currentMotionTypeConfig.limits.maxDecX = 4.0;
    vc.data.currentMotionTypeConfig.limits.maxDecY = 4.0;
    vc.data.currentMotionTypeConfig.limits.maxDecRz = 4.0;
    Velocity2D expectedVelocity;
    // face forward to make FCS and RCS coincide
    vc.data.robot.position.Rz = M_PI * 0.5;
    vc.data.target.pos.Rz = M_PI * 0.5;
    // fill in the values depending on degree of freedom
    if (dof == "X")
    {
        vc.data.robot.velocity.x = currentSpeed;
        vc.data.previousVelocityRcs.x = currentSpeed;
        vc.data.target.pos.x = newPosition;
        expectedVelocity.x = expectedSpeed;
    }
    if (dof == "Y")
    {
        vc.data.robot.velocity.y = currentSpeed;
        vc.data.previousVelocityRcs.y = currentSpeed;
        vc.data.target.pos.y = newPosition;
        expectedVelocity.y = expectedSpeed;
    }
    if (dof == "Rz")
    {
        vc.data.robot.velocity.Rz = currentSpeed;
        vc.data.previousVelocityRcs.phi = currentSpeed;
        vc.data.target.pos.Rz += newPosition;
        expectedVelocity.phi = expectedSpeed;
    }

    // Act
    vc.calculate();

    // Assert
    auto vel = vc.data.resultVelocityRcs;
    EXPECT_NEAR(vel.x, expectedVelocity.x, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.y, expectedVelocity.y, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.phi, expectedVelocity.phi, NUMERICAL_TOLERANCE);
}

// more limiter tests, inter-mixing XY

TEST(velocityControlTest, limiter_maxVelXY_positive)
{
    TRACE_FUNCTION("");

    // Arrange
    auto vc = defaultVelocityControlSetup();
    vc.data.currentMotionTypeConfig.velocityControllers.shortStroke.type = VelocitySetpointControllerTypeEnum::LINEAR;
    vc.data.currentMotionTypeConfig.velocityControllers.shortStroke.coordinateSystem = CoordinateSystemEnum::RCS;
    vc.data.vcSetpointConfig.type = VelocitySetpointControllerTypeEnum::LINEAR;
    vc.data.vcSetpointConfig.coordinateSystem = CoordinateSystemEnum::RCS;
    vc.data.target.pos.x = 2.0;
    vc.data.target.pos.y = 2.0;
    vc.data.currentMotionTypeConfig.limits.maxVelX = 0.50;
    vc.data.currentMotionTypeConfig.limits.maxVelYforward = 0.60;
    vc.data.currentMotionTypeConfig.limits.maxVelYbackward = 0.35;
    vc.data.currentMotionTypeConfig.limits.maxAccX = 200.0;
    vc.data.currentMotionTypeConfig.limits.maxAccYforward = 200.0;
    vc.data.currentMotionTypeConfig.limits.maxAccYbackward = 110.0;
    vc.data.currentMotionTypeConfig.limits.maxDecX = 200.0;
    vc.data.currentMotionTypeConfig.limits.maxDecY = 200.0;

    // Act
    vc.calculate();

    // Assert
    auto vel = vc.data.resultVelocityRcs;
    EXPECT_NEAR(vel.x, -0.50, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.y, 0.60, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.phi, 0.0, NUMERICAL_TOLERANCE);
}

TEST(velocityControlTest, limiter_maxVelXY_negative)
{
    TRACE_FUNCTION("");

    // Arrange
    auto vc = defaultVelocityControlSetup();
    vc.data.currentMotionTypeConfig.velocityControllers.shortStroke.type = VelocitySetpointControllerTypeEnum::LINEAR;
    vc.data.currentMotionTypeConfig.velocityControllers.shortStroke.coordinateSystem = CoordinateSystemEnum::RCS;
    vc.data.target.pos.x = -2.0;
    vc.data.target.pos.y = -2.0;
    vc.data.currentMotionTypeConfig.limits.maxVelX = 0.50;
    vc.data.currentMotionTypeConfig.limits.maxVelYforward = 0.60;
    vc.data.currentMotionTypeConfig.limits.maxVelYbackward = 0.60;
    vc.data.currentMotionTypeConfig.limits.maxAccX = 200.0;
    vc.data.currentMotionTypeConfig.limits.maxAccYforward = 200.0;
    vc.data.currentMotionTypeConfig.limits.maxAccYbackward = 200.0;
    vc.data.currentMotionTypeConfig.limits.maxDecX = 200.0;
    vc.data.currentMotionTypeConfig.limits.maxDecY = 200.0;

    // Act
    vc.calculate();

    // Assert
    auto vel = vc.data.resultVelocityRcs;
    EXPECT_NEAR(vel.x, 0.50, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.y, -0.60, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.phi, 0.0, NUMERICAL_TOLERANCE);
}

TEST(velocityControlTest, limiter_maxAccXY_positive)
{
    TRACE_FUNCTION("");

    // Arrange
    auto vc = defaultVelocityControlSetup();
    vc.data.target.pos.x = 2.0;
    vc.data.target.pos.y = 2.0;
    vc.data.currentMotionTypeConfig.limits.maxVelX = 10.0;
    vc.data.currentMotionTypeConfig.limits.maxVelYforward = 10.0;
    vc.data.currentMotionTypeConfig.limits.maxVelYbackward = 7.0;
    vc.data.currentMotionTypeConfig.limits.maxAccX = 40.0;
    vc.data.currentMotionTypeConfig.limits.maxAccYforward = 60.0;
    vc.data.currentMotionTypeConfig.limits.maxAccYbackward = 35.0;
    vc.data.currentMotionTypeConfig.limits.maxDecX = 70.0;
    vc.data.currentMotionTypeConfig.limits.maxDecY = 90.0;

    // Act
    vc.calculate();

    // Assert
    auto vel = vc.data.resultVelocityRcs;
    EXPECT_NEAR(vel.x, -2.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.y, 3.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.phi, 0.0, NUMERICAL_TOLERANCE);
}

TEST(velocityControlTest, limiter_maxAccXY_negative)
{
    TRACE_FUNCTION("");

    // Arrange
    auto vc = defaultVelocityControlSetup();
    vc.data.robot.velocity = pose(-0.1, -0.1, 0);
    vc.data.target.pos.x = -2.0;
    vc.data.target.pos.y = -2.0;
    vc.data.currentMotionTypeConfig.limits.maxVelX = 10.0;
    vc.data.currentMotionTypeConfig.limits.maxVelYforward = 10.0;
    vc.data.currentMotionTypeConfig.limits.maxVelYbackward = 7.0;
    vc.data.currentMotionTypeConfig.limits.maxAccX = 40.0;
    vc.data.currentMotionTypeConfig.limits.maxAccYforward = 25.0;
    vc.data.currentMotionTypeConfig.limits.maxAccYbackward = 40.0;

    // Act
    vc.calculate();

    // Assert
    auto vel = vc.data.resultVelocityRcs;
    EXPECT_NEAR(vel.x, 2.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.y, -2.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.phi, 0.0, NUMERICAL_TOLERANCE);
}

TEST(velocityControlTest, limiter_maxVelRz)
{
    TRACE_FUNCTION("");

    // Arrange
    auto vc = defaultVelocityControlSetup();
    vc.data.target.pos.Rz = 4.0;
    vc.data.currentMotionTypeConfig.limits.maxVelRz = 1.42;
    vc.data.currentMotionTypeConfig.limits.maxAccRz = 200.0;
    vc.data.currentMotionTypeConfig.limits.maxDecRz = 200.0;

    // Act
    vc.calculate();

    // Assert
    auto vel = vc.data.resultVelocityRcs;
    EXPECT_NEAR(vel.x, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.y, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.phi, -1.42, NUMERICAL_TOLERANCE);
}

TEST(velocityControlTest, limiter_maxAccRz)
{
    TRACE_FUNCTION("");

    // Arrange
    auto vc = defaultVelocityControlSetup();
    vc.data.target.pos.Rz = 4.0;
    vc.data.currentMotionTypeConfig.limits.maxVelRz = 10.0;
    vc.data.currentMotionTypeConfig.limits.maxAccRz = 5.0;
    vc.data.currentMotionTypeConfig.limits.maxDecRz = 5.0;

    // Act
    vc.calculate();

    // Assert
    auto vel = vc.data.resultVelocityRcs;
    EXPECT_NEAR(vel.x, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.y, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.phi, -0.25, NUMERICAL_TOLERANCE);
}

TEST(velocityControlTest, limiter_maxAccRz_breaking_positive)
{
    TRACE_FUNCTION("");

    // Arrange
    auto vc = defaultVelocityControlSetup();
    vc.data.target.pos.Rz = -2.0;
    vc.data.currentMotionTypeConfig.limits.maxVelRz = 10.0;
    vc.data.currentMotionTypeConfig.limits.maxAccRz = 5.0;
    vc.data.currentMotionTypeConfig.limits.maxDecRz = 5.0;
    vc.data.previousVelocityRcs = Velocity2D(0, 0, 1.0);

    // Act
    vc.calculate();

    // Assert
    auto vel = vc.data.resultVelocityRcs;
    EXPECT_NEAR(vel.x, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.y, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.phi, 0.75, NUMERICAL_TOLERANCE);
}

TEST(velocityControlTest, limiter_maxAccRz_breaking_negative)
{
    TRACE_FUNCTION("");

    // Arrange
    auto vc = defaultVelocityControlSetup();
    vc.data.target.pos.Rz = 2.0;
    vc.data.currentMotionTypeConfig.limits.maxVelRz = 10.0;
    vc.data.currentMotionTypeConfig.limits.maxAccRz = 5.0;
    vc.data.currentMotionTypeConfig.limits.maxDecRz = 5.0;
    vc.data.previousVelocityRcs = Velocity2D(0, 0, -1.0);

    // Act
    vc.calculate();

    // Assert
    auto vel = vc.data.resultVelocityRcs;
    EXPECT_NEAR(vel.x, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.y, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.phi, -0.75, NUMERICAL_TOLERANCE);
}

// miscellaneous

TEST(velocityControlTest, spg_workaround_necessity)
{
    TRACE_FUNCTION("");

    // Prepare
    auto vc = defaultVelocityControlSetup();
    vc.data.robot.position = pose(0, 0, 0);
    vc.data.target.pos = pose(0, 0, 0.004);
    vc.data.ppConfig.deadzone.toleranceRz = 0.001;
    vc.data.vcSetpointConfig.type = VelocitySetpointControllerTypeEnum::SPG;

    // Part 1: check that SPG without workaround does not converge
    vc.data.currentMotionTypeConfig.setPointGenerator.convergenceWorkaround = false;
    vc.calculate();
    auto vel = vc.data.resultVelocityRcs;
    EXPECT_EQ(vel.phi, 0.0); // if this is untrue, then we can probably remove the workaround from code?

    // Part 2: check that SPG with workaround actually produces a (small) setpoint
    vc.clearVelocitySetpointController(); // force SPG re-init with new config
    vc.data.vcSetpointConfig.type = VelocitySetpointControllerTypeEnum::SPG;
    vc.data.currentMotionTypeConfig.setPointGenerator.convergenceWorkaround = true;
    vc.calculate();
    vel = vc.data.resultVelocityRcs;
    EXPECT_NEAR(vel.phi, 0.08, NUMERICAL_TOLERANCE);
}


int main(int argc, char **argv)
{
    INIT_TRACE;
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    WRITE_TRACE;
    return r;
}
