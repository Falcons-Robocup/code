// Copyright 2019-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * pathPlanningTest.cpp
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
#include "pathPlanningTestDefaults.hpp"
//#include "MiniSimulation.hpp"
#include "tracing.hpp"

#define NUMERICAL_TOLERANCE 1e-4


TEST(pathPlanningTest, inactive_shouldFail)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = PathPlanning();
    pp.data.robot.status = robotStatusEnum::OUTOFPLAY;

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::FAILED);
    EXPECT_EQ(pp.data.stop, true);
}

TEST(pathPlanningTest, onTarget_shouldPass)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::PASSED);
    EXPECT_EQ(pp.data.stop, false);
    auto pos = pp.data.path.front().pos;
    EXPECT_EQ(pos.x, pp.data.robot.position.x);
    EXPECT_EQ(pos.y, pp.data.robot.position.y);
    EXPECT_EQ(pos.Rz, pp.data.robot.position.Rz);
    auto vel = pp.data.path.front().vel;
    EXPECT_NEAR(vel.x, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.y, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(vel.Rz, 0.0, NUMERICAL_TOLERANCE);
}

TEST(pathPlanningTest, simpleMove_shouldMoveForward)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    auto targetPos = pose(1, 0, 0);
    pp.data.target.pos = targetPos;

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::RUNNING);
    EXPECT_EQ(pp.data.stop, false);
    EXPECT_NEAR(pp.data.path.front().pos.x, targetPos.x, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(pp.data.path.front().pos.y, targetPos.y, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(pp.data.path.front().pos.Rz, targetPos.Rz, NUMERICAL_TOLERANCE);
}

TEST(pathPlanningTest, stop)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    pp.data.target.pos = pose(1, 0, 0);
    pp.data.stop = true;

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::PASSED);
    EXPECT_EQ(pp.data.stop, true);
}


TEST(pathPlanningTest, closebyXYnotRz_shouldContinue)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    pp.data.target.pos.Rz = 1.0;

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::RUNNING);
}

TEST(pathPlanningTest, closebyRznotXY_shouldContinue)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    pp.data.target.pos.y = 1.0;

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::RUNNING);
}


// boundary limiters

TEST(pathPlanningTest, targetOutOfBounds_shouldFail)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    pp.data.configPP.boundaries.targetOutsideField = BoundaryOptionEnum::STOP_AND_FAIL;
    pp.data.target.pos = pose(99, 99, 0);

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::FAILED);
    EXPECT_EQ(pp.data.stop, true);
}

TEST(pathPlanningTest, targetX_OutOfBounds_shouldClip)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    pp.data.configPP.boundaries.targetOutsideField = BoundaryOptionEnum::CLIP;
    pp.data.configPP.boundaries.fieldMarginX = 0.9; // TODO stub environmentField
    pp.data.target.pos = pose(99, 0, 0);

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::RUNNING);
    auto subtarget = pp.data.path.front();
    EXPECT_NEAR(subtarget.pos.x, 6.9, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(subtarget.pos.y, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(subtarget.pos.Rz, 0.0, NUMERICAL_TOLERANCE);
}

TEST(pathPlanningTest, targetY_OutOfBounds_shouldClip)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    pp.data.configPP.boundaries.targetOutsideField = BoundaryOptionEnum::CLIP;
    pp.data.configPP.boundaries.fieldMarginY = 0.4; // TODO stub environmentField
    pp.data.target.pos = pose(0, 99, 0);

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::RUNNING);
    auto subtarget = pp.data.path.front();
    EXPECT_NEAR(subtarget.pos.x, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(subtarget.pos.y, 9.4, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(subtarget.pos.Rz, 0.0, NUMERICAL_TOLERANCE);
}

TEST(pathPlanningTest, targetY_ownHalf_notAllowed)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    pp.data.configPP.boundaries.targetOnOwnHalf = BoundaryOptionEnum::STOP_AND_FAIL;
    pp.data.target.pos = pose(0.0, -6.0, 0.0);

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::FAILED);
    EXPECT_EQ(pp.data.stop, true);
}

TEST(pathPlanningTest, targetY_ownHalf_clip)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    pp.data.configPP.boundaries.targetOnOwnHalf = BoundaryOptionEnum::CLIP;
    pp.data.target.pos = pose(1.0, -6.0, 0.0);

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::RUNNING);
    auto subtarget = pp.data.path.front();
    EXPECT_NEAR(subtarget.pos.x, 1.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(subtarget.pos.y, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(subtarget.pos.Rz, 0.0, NUMERICAL_TOLERANCE);
}

TEST(pathPlanningTest, targetY_oppHalf_notAllowed)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    pp.data.configPP.boundaries.targetOnOpponentHalf = BoundaryOptionEnum::STOP_AND_FAIL;
    pp.data.target.pos = pose(0.0, 6.0, 0.0);

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::FAILED);
    EXPECT_EQ(pp.data.stop, true);
}

TEST(pathPlanningTest, targetY_oppHalf_clip)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    pp.data.configPP.boundaries.targetOnOpponentHalf = BoundaryOptionEnum::CLIP;
    pp.data.target.pos = pose(1.0, 6.0, 0.0);

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::RUNNING);
    auto subtarget = pp.data.path.front();
    EXPECT_NEAR(subtarget.pos.x, 1.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(subtarget.pos.y, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(subtarget.pos.Rz, 0.0, NUMERICAL_TOLERANCE);
}

// forbidden areas

TEST(pathPlanningTest, targetInForbiddenArea_shouldFail)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    pp.data.target.pos = pose(0.0, 6.0, 0.0);
    forbiddenArea f; // construct a forbidden area around the target (penalty marker)
    f.points.push_back(vec2d(-1.0,  5.0));
    f.points.push_back(vec2d(-1.0,  7.0));
    f.points.push_back(vec2d( 1.0,  7.0));
    f.points.push_back(vec2d( 1.0,  5.0));
    pp.data.forbiddenAreas.push_back(f);

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::FAILED);
    EXPECT_EQ(pp.data.stop, true);
}

TEST(pathPlanningTest, currentInForbiddenArea_shouldMoveOut)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    pp.data.robot.position = pose(0.0, 6.0, 0.0);
    forbiddenArea f; // construct a forbidden area around the target (penalty marker)
    f.points.push_back(vec2d(-1.0,  5.0));
    f.points.push_back(vec2d(-1.0,  7.0));
    f.points.push_back(vec2d( 1.0,  7.0));
    f.points.push_back(vec2d( 1.0,  5.0));
    pp.data.forbiddenAreas.push_back(f);

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::RUNNING);
    auto subtarget = pp.data.path.front();
    EXPECT_NEAR(subtarget.pos.x, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(subtarget.pos.y, 0.0, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(subtarget.pos.Rz, 0.0, NUMERICAL_TOLERANCE);
}

// obstacle avoidance

TEST(pathPlanningTest, avoid_teammember_left)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    pp.data.target.pos = pose(4.0, 0.1, 0.0);
    robotState r;
    r.position = pose(2.0, 0.0, 0.0);
    pp.data.teamMembers.push_back(r);

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::RUNNING);
    auto subtarget = pp.data.path.front();
    EXPECT_NEAR(subtarget.pos.x, 2.0, 0.5);
    EXPECT_NEAR(subtarget.pos.y, pp.data.configPP.obstacleAvoidance.subTargetDistance, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(subtarget.pos.Rz, 0.0, NUMERICAL_TOLERANCE);
}

TEST(pathPlanningTest, avoid_teammember_right)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    pp.data.target.pos = pose(4.0, -0.1, 0.0);
    robotState r;
    r.position = pose(2.0, 0.0, 0.0);
    pp.data.teamMembers.push_back(r);

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::RUNNING);
    auto subtarget = pp.data.path.front();
    EXPECT_NEAR(subtarget.pos.x, 2.0, 0.5);
    EXPECT_NEAR(subtarget.pos.y, -pp.data.configPP.obstacleAvoidance.subTargetDistance, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(subtarget.pos.Rz, 0.0, NUMERICAL_TOLERANCE);
}

TEST(pathPlanningTest, avoid_moving_teammember)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    pp.data.target.pos = pose(4.0, -0.3, 0.0);
    pp.data.target.vel = pose(0.0, 2.0, 0.0);
    robotState r;
    r.position = pose(2.0, 0.0, 0.0);
    pp.data.teamMembers.push_back(r);

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::RUNNING);
    auto subtarget = pp.data.path.front();
    EXPECT_NEAR(subtarget.pos.x, 2.0, 0.5);
    EXPECT_NEAR(subtarget.pos.y, -pp.data.configPP.obstacleAvoidance.subTargetDistance, NUMERICAL_TOLERANCE);
    EXPECT_NEAR(subtarget.pos.Rz, 0.0, NUMERICAL_TOLERANCE);
}

TEST(pathPlanningTest, avoid_obstacle)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    pp.data.target.pos = pose(4.0, 0.0, 0.0);
    obstacleResult obst;
    obst.position = vec2d(2.0, -0.1);
    pp.data.obstacles.push_back(obst);

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::RUNNING);
    auto subtarget = pp.data.path.front();
    EXPECT_NEAR(subtarget.pos.x, 2.0, 0.5);
    EXPECT_NEAR(subtarget.pos.y, obst.position.y + pp.data.configPP.obstacleAvoidance.subTargetDistance, 0.1);
    EXPECT_NEAR(subtarget.pos.Rz, 0.0, NUMERICAL_TOLERANCE);
}

TEST(pathPlanningTest, avoid_obstacle_cluster_left)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    pp.data.target.pos = pose(4.0, 0.0, 0.0);
    obstacleResult obst;
    obst.position = vec2d(1.5, -0.5); pp.data.obstacles.push_back(obst);
    obst.position = vec2d(1.5, -0.1); pp.data.obstacles.push_back(obst);
    obst.position = vec2d(1.5,  0.3); pp.data.obstacles.push_back(obst);
    obst.position = vec2d(2.0, -0.5); pp.data.obstacles.push_back(obst);
    obst.position = vec2d(2.0, -0.1); pp.data.obstacles.push_back(obst);
    obst.position = vec2d(2.0,  0.3); pp.data.obstacles.push_back(obst);
    obst.position = vec2d(2.5, -0.5); pp.data.obstacles.push_back(obst);
    obst.position = vec2d(2.5, -0.1); pp.data.obstacles.push_back(obst);
    obst.position = vec2d(2.5,  0.3); pp.data.obstacles.push_back(obst);

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::RUNNING);
    auto subtarget = pp.data.path.front();
    EXPECT_NEAR(subtarget.pos.x, 1.5, 0.5);
    EXPECT_NEAR(subtarget.pos.y, 1.0, 0.5);
    EXPECT_NEAR(subtarget.pos.Rz, 0.0, NUMERICAL_TOLERANCE);
}

TEST(pathPlanningTest, avoid_obstacle_cluster_right)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    pp.data.target.pos = pose(4.0, -1.0, 0.0);
    obstacleResult obst;
    obst.position = vec2d(1.5, -0.5); pp.data.obstacles.push_back(obst);
    obst.position = vec2d(1.5, -0.1); pp.data.obstacles.push_back(obst);
    obst.position = vec2d(1.5,  0.3); pp.data.obstacles.push_back(obst);
    obst.position = vec2d(2.0, -0.5); pp.data.obstacles.push_back(obst);
    obst.position = vec2d(2.0, -0.1); pp.data.obstacles.push_back(obst);
    obst.position = vec2d(2.0,  0.3); pp.data.obstacles.push_back(obst);
    obst.position = vec2d(2.5, -0.5); pp.data.obstacles.push_back(obst);
    obst.position = vec2d(2.5, -0.1); pp.data.obstacles.push_back(obst);
    obst.position = vec2d(2.5,  0.3); pp.data.obstacles.push_back(obst);

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::RUNNING);
    auto subtarget = pp.data.path.front();
    EXPECT_NEAR(subtarget.pos.x, 1.5, 0.5);
    EXPECT_NEAR(subtarget.pos.y, -1.0, 0.5);
    EXPECT_NEAR(subtarget.pos.Rz, 0.0, NUMERICAL_TOLERANCE);
}

/*
TEST(pathPlanningTest, avoid_obstacles_jailed) // TODO: disabled for now; undefined behavior.. robot should perhaps just stop?
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    pp.data.target.pos = pose(4.0, 0.0, 0.0);
    obstacleResult obst;
    obst.position = vec2d(-0.4, -0.4); pp.data.obstacles.push_back(obst);
    obst.position = vec2d(-0.4,  0.4); pp.data.obstacles.push_back(obst);
    obst.position = vec2d( 0.4,  0.4); pp.data.obstacles.push_back(obst);
    obst.position = vec2d( 0.4, -0.4); pp.data.obstacles.push_back(obst);

    // Iteration 1: robot exactly at center, move towards target
    pp.data.robot.position = pose(0.0, 0.0, 0.0);
    auto result = pp.calculate();
    EXPECT_EQ(result, actionResultTypeEnum::RUNNING);
    auto subtarget = pp.data.path[0];
    EXPECT_NEAR(subtarget.pos.x, 0.4, 0.1);
    EXPECT_NEAR(subtarget.pos.y, 0.4, 0.1); // ! robot actually moves towards one of the obstacles ... TODO?
    EXPECT_NEAR(subtarget.pos.Rz, 0.0, NUMERICAL_TOLERANCE);

    // Iteration 2: robot on its way
    pp.data.robot.position = pose(0.2, 0.0, 0.0);
    result = pp.calculate();
    EXPECT_EQ(result, actionResultTypeEnum::RUNNING);
    subtarget = pp.data.path[0];
    EXPECT_NEAR(subtarget.pos.x, 0.6, 0.2);
    EXPECT_NEAR(subtarget.pos.y, 0.0, 0.2);
    EXPECT_NEAR(subtarget.pos.Rz, 0.0, NUMERICAL_TOLERANCE);
}
*/

// Tests to check yaml configurations: can robot find a way between two perfect still obstacles
// as is required for TCrun2 (where noise comes into play as well)
// First parameter: yaml file
// Second parameter: distance between obstacles
// Third parameter: y offset of obstacles
class ObstacleDriveThroughTest : public testing::TestWithParam<std::tuple<std::string, float, float>>
{
public:
    virtual void SetUp(){}
    virtual void TearDown(){}
};

INSTANTIATE_TEST_CASE_P(pathPlanningTest,
                        ObstacleDriveThroughTest,
                        ::testing::Values(
                            std::make_tuple("PathPlanningSim.yaml", 1.8,  0.0), // perfect placement, no noise
                            std::make_tuple("PathPlanningSim.yaml", 1.4,  0.0), // worst case placement, no noise
                            std::make_tuple("PathPlanningSim.yaml", 1.2,  0.0), // worst case placement, allow a bit of vision/worldModel noise
                            std::make_tuple("PathPlanning.yaml",    1.8,  0.0), // perfect placement, no noise
                            std::make_tuple("PathPlanning.yaml",    1.4,  0.0), // worst case placement, no noise
                            // repeat all tests, but with offsets, to stress-test angle calculations (left/right orientation)
                            std::make_tuple("PathPlanningSim.yaml", 1.8,  0.3),
                            std::make_tuple("PathPlanningSim.yaml", 1.4,  0.3),
                            std::make_tuple("PathPlanningSim.yaml", 1.2,  0.3),
                            std::make_tuple("PathPlanning.yaml",    1.8,  0.3),
                            std::make_tuple("PathPlanning.yaml",    1.4,  0.3),
                            std::make_tuple("PathPlanningSim.yaml", 1.8, -0.3),
                            std::make_tuple("PathPlanningSim.yaml", 1.4, -0.3),
                            std::make_tuple("PathPlanningSim.yaml", 1.2, -0.3),
                            std::make_tuple("PathPlanning.yaml",    1.8, -0.3),
                            std::make_tuple("PathPlanning.yaml",    1.4, -0.3)
                            )
                        );

// NOTE: the 1.2m distance tests using PathPlanning.yaml have been removed as they were sensitive to tuning

TEST_P(ObstacleDriveThroughTest, ObstacleDriveThroughTests)
{
    const ::testing::TestInfo* const test_info = ::testing::UnitTest::GetInstance()->current_test_info();
    TRACE_FUNCTION(test_info->name());

    // Parameters
    std::string yamlfile = std::get<0>(GetParam());
    float distanceBetweenObstacles = std::get<1>(GetParam());
    float offsetY = std::get<2>(GetParam());
    TRACE("test parameters: yamlfile=%s distanceBetweenObstacles=%6.2f offsetY=%6.2f", yamlfile.c_str(), distanceBetweenObstacles, offsetY);

    // Arrange
    auto pp = yamlPathPlanningSetup(yamlfile);
    pp.data.configPP.obstacleAvoidance.enabled = true; // make sure it is enabled, yaml might temporarily use false
    pp.data.configPP.forwardDriving.withoutBall.enabled = false; // prevent extra sub-targets
    EXPECT_GT(pp.data.configPP.obstacleAvoidance.subTargetDistance, 0.1); // sanity check if YAML was properly loaded
    pp.data.robot.position.y = offsetY;
    pp.data.target.pos = pose(4.0, offsetY, 0.0);
    obstacleResult obst;
    obst.position = vec2d(2.0, -0.5 * distanceBetweenObstacles); pp.data.obstacles.push_back(obst);
    obst.position = vec2d(2.0,  0.5 * distanceBetweenObstacles); pp.data.obstacles.push_back(obst);

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::RUNNING);
    auto subtarget = pp.data.path.front();
    // either no extra subtarget (= no obstacle avoidance)
    // or a subtarget inbetween both obstacles
    if (subtarget.pos.x > 3.0)
    {
        EXPECT_NEAR(subtarget.pos.x, 4.0, NUMERICAL_TOLERANCE);
        EXPECT_NEAR(subtarget.pos.y, offsetY, NUMERICAL_TOLERANCE);
    }
    else
    {
        EXPECT_NEAR(subtarget.pos.x, 2.0, 0.2);
        EXPECT_NEAR(subtarget.pos.y, 0.0, 0.25 * distanceBetweenObstacles);
    }
}

TEST(pathPlanningTest, avoid_forbidden_area_left)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    pp.data.target.pos = pose(4.0, 0.3, 0.0);
    forbiddenArea f; // construct a forbidden area in between robot and target
    f.points.push_back(vec2d(1.0, -1.0));
    f.points.push_back(vec2d(1.0,  1.0));
    f.points.push_back(vec2d(3.0,  1.0));
    f.points.push_back(vec2d(3.0, -1.0));
    pp.data.forbiddenAreas.push_back(f);

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::RUNNING);
    auto subtarget = pp.data.path.front();
    EXPECT_NEAR(subtarget.pos.x, 1.0, 0.8);
    EXPECT_NEAR(subtarget.pos.y, 1.0, 0.8);
    EXPECT_NEAR(subtarget.pos.Rz, 0.0, NUMERICAL_TOLERANCE);
}

TEST(pathPlanningTest, avoid_forbidden_area_right)
{
    TRACE_FUNCTION("");

    // Arrange
    auto pp = defaultPathPlanningSetup();
    pp.data.target.pos = pose(4.0, -0.3, 0.0);
    forbiddenArea f; // construct a forbidden area in between robot and target
    f.points.push_back(vec2d(1.0, -1.0));
    f.points.push_back(vec2d(1.0,  1.0));
    f.points.push_back(vec2d(3.0,  1.0));
    f.points.push_back(vec2d(3.0, -1.0));
    pp.data.forbiddenAreas.push_back(f);

    // Act
    auto result = pp.calculate();

    // Assert
    EXPECT_EQ(result, actionResultTypeEnum::RUNNING);
    auto subtarget = pp.data.path.front();
    EXPECT_NEAR(subtarget.pos.x, 1.0, 0.8);
    EXPECT_NEAR(subtarget.pos.y, -1.0, 0.8);
    EXPECT_NEAR(subtarget.pos.Rz, 0.0, NUMERICAL_TOLERANCE);
}


// 2020-10-23, EKPC: Disable MiniSimulationTest after splitting PathPlanning / VelocityControl.
// The output of PathPlanning is now a subtarget, so this no longer fits in MiniSimulation.
// Besides, reinventing simulation for testing purposes is way out of scope for the PathPlanning tests.
// The idea is really good, but should use the existing simulation package, and should be considered an system level / integration test.

//// Mini simulation tests, to verify closed-loop behavior over time.
//
//// yaml loading is a bit expensive -> cache
//static ConfigPathPlanning ppCfgSim     = loadYAML("PathPlanningSim.yaml");
//static ConfigPathPlanning ppCfgReal    = adaptConfigToSim(loadYAML("PathPlanning.yaml"));
//
//// TokyoDrift variants
//ConfigPathPlanning withBallLimiters(ConfigPathPlanning cfg)
//{
//    cfg.forwardDriving.applyLimitsToBall = true;
//    return cfg;
//}
//static ConfigPathPlanning ppCfgSimT    = withBallLimiters(ppCfgSim);
//static ConfigPathPlanning ppCfgRealT   = withBallLimiters(ppCfgReal);
//
//// DiagonalTest variants
//ConfigPathPlanning setForwardDriving(ConfigPathPlanning cfg, bool forwardDriving)
//{
//    cfg.forwardDriving.withoutBall.enabled = forwardDriving;
//    return cfg;
//}
//ConfigPathPlanning setSynchronizeRotation(ConfigPathPlanning cfg, bool synchronizeRotation)
//{
//    cfg.setPointGenerator.synchronizeRotation = synchronizeRotation;
//    return cfg;
//}
//static ConfigPathPlanning ppCfgSimFS   = setSynchronizeRotation(setForwardDriving(ppCfgSim, true), true);
//static ConfigPathPlanning ppCfgSimNFS  = setSynchronizeRotation(setForwardDriving(ppCfgSim, false), true);
//static ConfigPathPlanning ppCfgSimFNS  = setSynchronizeRotation(setForwardDriving(ppCfgSim, true), false);
//static ConfigPathPlanning ppCfgSimNFNS = setSynchronizeRotation(setForwardDriving(ppCfgSim, false), false);
//
//// every mini simulation end with evaluating and 'expectation'
//
//typedef std::pair<float, float> ExpectationRange;
//
//#define EXPECT_INRANGE(value, range) 
//    EXPECT_GE(value + NUMERICAL_TOLERANCE, range.first); 
//    EXPECT_LE(value - NUMERICAL_TOLERANCE, range.second);
//
//struct MiniSimulationExpectation
//{
//    actionResultTypeEnum finalResult = actionResultTypeEnum::PASSED;
//    ExpectationRange durationRange = std::make_pair(0.1, 20.0);
//    ExpectationRange distanceRange = std::make_pair(0.1, 20.0);
//    ExpectationRange maxSpeedRobotRange = std::make_pair(0.1, 3.0);
//    ExpectationRange maxVelBallXRange = std::make_pair(0.0, 20.0);
//    ExpectationRange maxVelBallYRange = std::make_pair(0.0, 40.0);
//    ExpectationRange maxAccBallXRange = std::make_pair(0.0, 20.0);
//    ExpectationRange maxAccBallYRange = std::make_pair(0.0, 40.0);
//    ExpectationRange minDistanceToObstaclesRange = std::make_pair(0.5, 100.0);
//
//    MiniSimulationExpectation() {}
//
//    void evaluate(MiniSimulationResult const &mr)
//    {
//        EXPECT_EQ(mr.success, true);
//        EXPECT_EQ(mr.status, finalResult);
//        EXPECT_INRANGE(mr.duration, durationRange);
//        EXPECT_INRANGE(mr.distance, distanceRange);
//        EXPECT_INRANGE(mr.maxSpeedRobot, maxSpeedRobotRange);
//        EXPECT_INRANGE(mr.maxVelBall.x, maxVelBallXRange);
//        EXPECT_INRANGE(mr.maxVelBall.y, maxVelBallYRange);
//        EXPECT_INRANGE(mr.maxAccBall.x, maxAccBallXRange);
//        EXPECT_INRANGE(mr.maxAccBall.y, maxAccBallYRange);
//        EXPECT_INRANGE(mr.minDistanceToObstacles, minDistanceToObstaclesRange);
//    }
//};
//
//// Mini simulation test instantiation
//
//// shorthands for readibilty
//#define VCT  VelocitySetpointControllerTypeEnum
//#define VCCS CoordinateSystemEnum
//
//// Test parameters:
//// * simulated scene
//// * configuration
//// * overrule velocity setpoint controller type (PID, LINEAR, ..)
//// * overrule velocity setpoint controller coordinate system (FCS, RCS)
//// * expected results
//class MiniSimulationTest : public testing::TestWithParam<std::tuple<PathPlanningSimulationScene, ConfigPathPlanning, VelocitySetpointControllerTypeEnum, CoordinateSystemEnum, MiniSimulationExpectation>>
//{
//public:
//    virtual void SetUp(){}
//    virtual void TearDown(){}
//};
//
//PathPlanningSimulationScene avoidWithBallScene()
//{
//    PathPlanningSimulationScene result;
//    result.robotId = MINISIMULATION_DEFAULT_ROBOT_ID;
//    result.hasBall = true;
//    result.target.pos = pose(4.0, 0.0, 1.0);
//    result.target.vel = pose(0.0, 0.0, 0.0);
//    SimulationSceneRobot r;
//    r.robotId = result.robotId;
//    r.position = pose(0.0, 0.0, 1.0);
//    r.velocity = pose(0.0, 0.0, 0.0);
//    result.robots.push_back(r);
//    SimulationSceneObstacle obst;
//    obst.position = vec2d(2.0, 0.0);
//    obst.velocity = vec2d(0.0, 0.0);
//    result.obstacles.push_back(obst);
//    return result;
//}
//
//MiniSimulationExpectation expectationAvoidWithBall()
//{
//    MiniSimulationExpectation result;
//    result.finalResult = actionResultTypeEnum::PASSED;
//    result.durationRange.second = 10.0;
//    result.distanceRange.first  =  4.1;
//    result.distanceRange.second = 10.0;
//    return result;
//}
//
//PathPlanningSimulationScene pathFindingScene()
//{
//    // stress test, to check for infinite loops / poor runtime performance (this used to be a thing once)
//    PathPlanningSimulationScene result;
//    result.robotId = MINISIMULATION_DEFAULT_ROBOT_ID;
//    result.hasBall = false;
//    result.target.pos = pose(6.0, 9.0, 0.0);
//    result.target.vel = pose(0.0, 0.0, 0.0);
//    SimulationSceneRobot r;
//    r.robotId = result.robotId;
//    r.position = pose(0.0, -2.0, 0.0);
//    r.velocity = pose(0.0, 0.0, 0.0);
//    result.robots.push_back(r);
//
///*
//legend: T is target, R is robot starting position
//x: forbidden areas
//o: intended path
//1,2,3,4: teammembers
//A,B,C,...: obstacles
//note that robot will not choose the shortest path, due to the greedy nature of the algorithm
//
//    9                           ooT
//         xxx   xxx   xxx   xxx o xxx
//    8    xxx   xxx   xxx   xxx o xxx
//         xxx   xxx   xxx   xxx o xxx
//    7                          o
//         xxx   xxx   xxx   xxx o xxx
//    6    xxx   xxx   xxx   xxx o xxx
//         xxx   xxx   xxx   xxx o xxx
//    5                          o
//         xxx   xxx   xxx   xxx o xxx
//    4    xxA--->xB--->xC--->xx o xxx
//         xEx   xxx   xxx   xDx o xxx
//    3     |   ooooooooooo   |  o
//         x|x o xxx   xxx o x|x o xxx
//    2    xvx o xx<---4^x o xvx o xxx
//         xxx o xxx   x|x o xxx o xxx
//    1         o       |   ooooo
//         xxx   o     x1x   xxx   xxx
//  y 0    xxx    R2   xx3--->x5--->xx
//         xxx         xxx   xxx   xxx
//   -1
//      -3 -2 -1  0  1  2  3  4  5  6
//                x
//*/
//
//    // construct several forbidden areas on a grid
//    float forbiddenAreaSize = 0.8;
//    for (int ix = -1; ix <= 3; ++ix)
//    {
//        for (int iy = 0; iy <= 4; ++iy)
//        {
//            if (ix != 0 || iy != 0)
//            {
//                forbiddenArea f;
//                f.points.push_back(vec2d(2.0*ix - 0.5*forbiddenAreaSize, 2.0*iy - 0.5*forbiddenAreaSize));
//                f.points.push_back(vec2d(2.0*ix - 0.5*forbiddenAreaSize, 2.0*iy + 0.5*forbiddenAreaSize));
//                f.points.push_back(vec2d(2.0*ix + 0.5*forbiddenAreaSize, 2.0*iy + 0.5*forbiddenAreaSize));
//                f.points.push_back(vec2d(2.0*ix + 0.5*forbiddenAreaSize, 2.0*iy - 0.5*forbiddenAreaSize));
//                f.id = (int)result.forbiddenAreas.size();
//                result.forbiddenAreas.push_back(f);
//            }
//        }
//    }
//    // add a full team of teammembers, with velocities
//    // such that they 'connect' some forbidden areas
//    float v = 1.7;
//    r.robotId = 1;
//    r.position = pose(2.0, 0.5, M_PI*0.5);
//    r.velocity = pose(0.0, v, 0.0);
//    result.robots.push_back(r);
//    r.robotId = 3;
//    r.position = pose(2.5, 0.0, 0.0);
//    r.velocity = pose(v, 0.0, 0.0);
//    result.robots.push_back(r);
//    r.robotId = 4;
//    r.position = pose(1.5, 2.0, 0.0);
//    r.velocity = pose(-v, 0.0, 0.0);
//    result.robots.push_back(r);
//    r.robotId = 5;
//    r.position = pose(4.5, 0.0, 0.0);
//    r.velocity = pose(v, 0.0, 0.0);
//    result.robots.push_back(r);
//
//    // insert some obstacles with velocities,
//    // also such that they 'connect' some forbidden areas
//    SimulationSceneObstacle obst;
//    obst.position = vec2d(-1.5, 4.0);
//    obst.velocity = vec2d(v, 0.0);
//    result.obstacles.push_back(obst); // A
//    obst.position = vec2d(0.5, 4.0);
//    obst.velocity = vec2d(v, 0.0);
//    result.obstacles.push_back(obst); // B
//    obst.position = vec2d(2.5, 4.0);
//    obst.velocity = vec2d(v, 0.0);
//    //result.obstacles.push_back(obst); // C
//    obst.position = vec2d(4.0, 3.5);
//    obst.velocity = vec2d(0.0, -v);
//    //result.obstacles.push_back(obst); // D
//    obst.position = vec2d(-2.0, 3.5);
//    obst.velocity = vec2d(0.0, -v);
//    //result.obstacles.push_back(obst); // E
//
//    return result;
//}
//
//MiniSimulationExpectation expectationPathFinding()
//{
//    MiniSimulationExpectation result;
//    result.finalResult = actionResultTypeEnum::PASSED;
//    result.durationRange.second = 40.0;
//    result.distanceRange.first =  10.0;
//    result.distanceRange.second = 40.0;
//    return result;
//}
//
//PathPlanningSimulationScene tokyoDriftScene()
//{
//    PathPlanningSimulationScene result;
//    result.robotId = MINISIMULATION_DEFAULT_ROBOT_ID;
//    result.hasBall = true;
//    result.target.pos = pose(2.0, 0.0, 3.0);
//    result.target.vel = pose(0.0, 0.0, 0.0);
//    SimulationSceneRobot r;
//    r.robotId = result.robotId;
//    r.position = pose(0.0, 0.0, 0.0);
//    r.velocity = pose(0.0, 0.0, 0.0);
//    result.robots.push_back(r);
//    return result;
//}
//
//MiniSimulationExpectation expectationSmoothBall(bool sim = true)
//{
//    MiniSimulationExpectation result;
//    result.finalResult = actionResultTypeEnum::PASSED;
//    result.durationRange.second =  4.0;
//    result.distanceRange.first  =  1.9;
//    result.distanceRange.second =  2.5;
//    auto cfg = (sim ? ppCfgSim : ppCfgReal);
//    result.maxVelBallXRange.second = cfg.limits.withBall.maxVelX;
//    result.maxVelBallYRange.second = cfg.limits.withBall.maxVelYforward;
//    result.maxAccBallXRange.second = cfg.limits.withBall.maxAccX;
//    result.maxAccBallYRange.second = cfg.limits.withBall.maxAccYforward;
//    // workaround for calculated acceleration in MiniSumulation not being very robust yet
//    result.maxAccBallXRange.second = 99;
//    result.maxAccBallYRange.second = 99;
//    return result;
//}
//
//PathPlanningSimulationScene noisyLongMoveScene()
//{
//    PathPlanningSimulationScene result;
//    result.robotId = MINISIMULATION_DEFAULT_ROBOT_ID;
//    result.hasBall = false;
//    result.target.pos = pose(0.0, 8.0, 1.5);
//    result.target.vel = pose(0.0, 0.0, 0.0);
//    srand(0);
//    result.localizationNoise.xy = 0.01;
//    result.localizationNoise.Rz = 0.05;
//    return result;
//}
//
//PathPlanningSimulationScene diagonalMoveScene()
//{
//    PathPlanningSimulationScene result;
//    result.robotId = MINISIMULATION_DEFAULT_ROBOT_ID;
//    result.hasBall = false;
//    result.target.pos = pose(2.0, 4.0, 0.0);
//    result.target.vel = pose(0.0, 0.0, 0.0);
//    return result;
//}
//
//MiniSimulationExpectation expectationDiagonalMove()
//{
//    MiniSimulationExpectation result;
//    result.finalResult = actionResultTypeEnum::PASSED;
//    result.durationRange.second =  5.0;
//    result.distanceRange.first  =  4.2;
//    result.distanceRange.second =  4.6;
//    // optimal move: math.sqrt(2*2+4*4)   = 4.47
//    // bad move:     2+math.sqrt(2*2+2*2) = 4.83
//    return result;
//}
//
//INSTANTIATE_TEST_CASE_P(pathPlanningTest, MiniSimulationTest, ::testing::Values(
//    std::make_tuple(avoidWithBallScene(), ppCfgSim,   VCT::PID,    VCCS::FCS, expectationAvoidWithBall()),
//    std::make_tuple(avoidWithBallScene(), ppCfgSim,   VCT::PID,    VCCS::RCS, expectationAvoidWithBall()),
//    std::make_tuple(avoidWithBallScene(), ppCfgSim,   VCT::LINEAR, VCCS::FCS, expectationAvoidWithBall()),
//    std::make_tuple(avoidWithBallScene(), ppCfgSim,   VCT::LINEAR, VCCS::RCS, expectationAvoidWithBall()),
//    std::make_tuple(avoidWithBallScene(), ppCfgSim,   VCT::SPG,    VCCS::RCS, expectationAvoidWithBall()),
//    std::make_tuple(avoidWithBallScene(), ppCfgReal,  VCT::PID,    VCCS::FCS, expectationAvoidWithBall()),
//    std::make_tuple(avoidWithBallScene(), ppCfgReal,  VCT::PID,    VCCS::RCS, expectationAvoidWithBall()),
//    std::make_tuple(avoidWithBallScene(), ppCfgReal,  VCT::LINEAR, VCCS::FCS, expectationAvoidWithBall()),
//    std::make_tuple(avoidWithBallScene(), ppCfgReal,  VCT::LINEAR, VCCS::RCS, expectationAvoidWithBall()),
//    std::make_tuple(avoidWithBallScene(), ppCfgReal,  VCT::SPG,    VCCS::RCS, expectationAvoidWithBall()),
//    std::make_tuple(pathFindingScene(),   ppCfgSim,   VCT::LINEAR, VCCS::FCS, expectationPathFinding()),
//    std::make_tuple(pathFindingScene(),   ppCfgSim,   VCT::PID,    VCCS::FCS, expectationPathFinding()),
//    std::make_tuple(pathFindingScene(),   ppCfgReal,  VCT::PID,    VCCS::FCS, expectationPathFinding()),
//    std::make_tuple(pathFindingScene(),   ppCfgSim,   VCT::SPG,    VCCS::RCS, expectationPathFinding()),
//    std::make_tuple(pathFindingScene(),   ppCfgReal,  VCT::SPG,    VCCS::RCS, expectationPathFinding()),
//    std::make_tuple(diagonalMoveScene(),  ppCfgSimFS,   VCT::SPG,    VCCS::RCS, expectationDiagonalMove()),
//    std::make_tuple(diagonalMoveScene(),  ppCfgSimNFS,  VCT::SPG,    VCCS::RCS, expectationDiagonalMove()),
//    std::make_tuple(diagonalMoveScene(),  ppCfgSimFNS,  VCT::SPG,    VCCS::RCS, expectationDiagonalMove()),
//    std::make_tuple(diagonalMoveScene(),  ppCfgSimNFNS, VCT::SPG,    VCCS::RCS, expectationDiagonalMove()), // this test now FAILS
//    std::make_tuple(tokyoDriftScene(),    ppCfgSimT,  VCT::PID,    VCCS::RCS, expectationSmoothBall(true)),
//    std::make_tuple(tokyoDriftScene(),    ppCfgSimT,  VCT::SPG,    VCCS::RCS, expectationSmoothBall(true)),
//    std::make_tuple(tokyoDriftScene(),    ppCfgRealT, VCT::PID,    VCCS::RCS, expectationSmoothBall(false)),
//    std::make_tuple(tokyoDriftScene(),    ppCfgRealT, VCT::SPG,    VCCS::RCS, expectationSmoothBall(false)),
//    std::make_tuple(noisyLongMoveScene(), ppCfgReal,  VCT::PID,    VCCS::FCS, MiniSimulationExpectation()),
//    std::make_tuple(noisyLongMoveScene(), ppCfgReal,  VCT::PID,    VCCS::RCS, MiniSimulationExpectation())/*,
//    std::make_tuple(loadScene("overshootX.scene"),  "PathPlanningSim.yaml", ExpectationNoOvershoot()),
//    std::make_tuple(loadScene("overshootX.scene"),  "PathPlanning.yaml",    ExpectationNoOvershoot()),
//    std::make_tuple(loadScene("overshootY.scene"),  "PathPlanningSim.yaml", ExpectationNoOvershoot()),
//    std::make_tuple(loadScene("overshootY.scene"),  "PathPlanning.yaml",    ExpectationNoOvershoot()),
//    std::make_tuple(loadScene("overshootRz.scene"), "PathPlanningSim.yaml", ExpectationNoOvershoot()),
//    std::make_tuple(loadScene("overshootRz.scene"), "PathPlanning.yaml",    ExpectationNoOvershoot())
//    TODO: overshoot tests only make sense if we have some kind of simulated inertia -> roadmap RobotModel?
//    */
//    ));
//
//
//TEST_P(MiniSimulationTest, MiniSimulationTests)
//{
//    const ::testing::TestInfo* const test_info = ::testing::UnitTest::GetInstance()->current_test_info();
//    TRACE_FUNCTION(test_info->name());
//
//    // Parameters
//    PathPlanningSimulationScene scene = std::get<0>(GetParam());
//    ConfigPathPlanning config = std::get<1>(GetParam());
//    VelocitySetpointControllerTypeEnum vct = std::get<2>(GetParam());
//    CoordinateSystemEnum vccs = std::get<3>(GetParam());
//    MiniSimulationExpectation expectation = std::get<4>(GetParam());
//
//    // workaround for PID not converging fast enough in MiniSimulation (timeouts)
//    // kstplot analysis shows that overshoot can occur in simulation, and recovery takes too long
//    // root cause seems to be a combination of:
//    // * not simulating inertia
//    // * a nonzero PID integral tuning factor
//    // * tight tolerances
//    // we do not want our tests to be very sensitive to yaml (changes), hence this workaround
//    if (vct == VelocitySetpointControllerTypeEnum::PID)
//    {
//        config.pid.XY_I = 0.0;
//        config.pid.RZ_I = 0.0;
//    }
//
//    // Setup
//    MiniSimulation m(config, 45.0, true);
//    m.setRdlFilename("/var/tmp/" + boost::replace_all_copy(std::string(test_info->name()), "/", "_") + ".rdl");
//    m.setScene(scene);
//    m.initialize();
//    // overrule velocity setpoint controller
//    m.overruleVelocityController(vct, vccs);
//
//    // Run simulation
//    auto result = m.run();
//
//    // Evaluate results
//    expectation.evaluate(result);
//}

int main(int argc, char **argv)
{
    INIT_TRACE("pathPlanningTest");
    InitGoogleTest(&argc, argv);
    int r = RUN_ALL_TESTS();
    WRITE_TRACE;
    return r;
}
