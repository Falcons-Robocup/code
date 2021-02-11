// Copyright 2016 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Integration tests for Visualization class.
 * VisualizationTests.cpp
 *
 *  Created on: December 7th, 2016
 *      Author: Diana Koenraadt
 */
#include <gtest/gtest.h>

#include "include/int/widgets/Field/Visualization/RobotVisualization.h"
#include "PlannedPathStub.h"

TEST(RobotVisualization, setPath_signalSetPathCalled)
{
    // Arrange
    vtkSmartPointer<RobotVisualization> visualization = vtkSmartPointer<RobotVisualization>::New();
    visualization->setPathPlanningEnabled(true);
    PlannedPathStub stub;
    stub.initialize(visualization);

    std::vector<PositionVelocity> path;
    // Act 
    visualization->setPath(path);

    // Assert
    ASSERT_TRUE(stub.onPathChangedCalled);
}

TEST(RobotVisualization, setPath_signalCalledWithPath)
{
    // Arrange
    vtkSmartPointer<RobotVisualization> visualization = vtkSmartPointer<RobotVisualization>::New();
    visualization->setPathPlanningEnabled(true);
    PlannedPathStub stub;
    stub.initialize(visualization);

    std::vector<PositionVelocity> path;
    path.push_back(PositionVelocity(1,2,3,4,5,6,7,8));
    path.push_back(PositionVelocity(8,7,6,5,4,3,2,1));
    path.push_back(PositionVelocity(1,1,1,1,1,1,1,1));
    // Act 
    visualization->setPath(path);

    // Assert
    ASSERT_EQ(3, stub._path.size());

    ASSERT_EQ(1, stub._path[0].x);
    ASSERT_EQ(2, stub._path[0].y);
    ASSERT_EQ(3, stub._path[0].z);
    ASSERT_EQ(4, stub._path[0].phi);
    ASSERT_EQ(5, stub._path[0].vx);
    ASSERT_EQ(6, stub._path[0].vy);
    ASSERT_EQ(7, stub._path[0].vz);
    ASSERT_EQ(8, stub._path[0].vphi);

    ASSERT_EQ(8, stub._path[1].x);
    ASSERT_EQ(7, stub._path[1].y);
    ASSERT_EQ(6, stub._path[1].z);
    ASSERT_EQ(5, stub._path[1].phi);
    ASSERT_EQ(4, stub._path[1].vx);
    ASSERT_EQ(3, stub._path[1].vy);
    ASSERT_EQ(2, stub._path[1].vz);
    ASSERT_EQ(1, stub._path[1].vphi);

    ASSERT_EQ(1, stub._path[2].x);
    ASSERT_EQ(1, stub._path[2].y);
    ASSERT_EQ(1, stub._path[2].z);
    ASSERT_EQ(1, stub._path[2].phi);
    ASSERT_EQ(1, stub._path[2].vx);
    ASSERT_EQ(1, stub._path[2].vy);
    ASSERT_EQ(1, stub._path[2].vz);
    ASSERT_EQ(1, stub._path[2].vphi);
}

TEST(RobotVisualization, setPathPlanningEnabledToFalse_signalSetPathCalled)
{
    // Arrange
    vtkSmartPointer<RobotVisualization> visualization = vtkSmartPointer<RobotVisualization>::New();
    visualization->setPathPlanningEnabled(true);
    PlannedPathStub stub;
    stub.initialize(visualization);

    // Act 
    visualization->setPathPlanningEnabled(false);

    // Assert
    ASSERT_TRUE(stub.onPathChangedCalled);
}

TEST(RobotVisualization, setPathPlanningEnabledToFalse_signalSetPathCalledClearPath)
{
    // Arrange
    vtkSmartPointer<RobotVisualization> visualization = vtkSmartPointer<RobotVisualization>::New();
    visualization->setPathPlanningEnabled(true);
    PlannedPathStub stub;
    stub.initialize(visualization);

    std::vector<PositionVelocity> path;
    path.push_back(PositionVelocity(1,2,3,4,5,6,7,8));
    path.push_back(PositionVelocity(8,7,6,5,4,3,2,1));
    path.push_back(PositionVelocity(1,1,1,1,1,1,1,1));
    visualization->setPath(path);

    // Act 
    visualization->setPathPlanningEnabled(false);

    // Assert
    ASSERT_EQ(0, stub._path.size());
}

/*
 * Main entry
 */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
