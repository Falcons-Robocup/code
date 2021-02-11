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

#include "include/int/widgets/Field/Visualization/Visualization.h"
#include "AnnotationStub.h"

class MyVisualization : public Visualization
{
public:
    static MyVisualization* New()
    {
        return new MyVisualization();
    }

    void initialize(int robotID, vtkRenderer *renderer);
};

TEST(Visualization, setPosition_signalPositionChangedCalled)
{
    // Arrange
    vtkSmartPointer<MyVisualization> visualization = vtkSmartPointer<MyVisualization>::New();
    AnnotationStub stub;
    stub.initialize(visualization);

    // Act 
    PositionVelocity posvel;
    visualization->setPosition(posvel);

    // Assert
    ASSERT_TRUE(stub.onAnchorPositionChangedCalled);
}

TEST(Visualization, setPosition_signalPositionChangedCalledWithPositionVelocity)
{
    // Arrange
    vtkSmartPointer<MyVisualization> visualization = vtkSmartPointer<MyVisualization>::New();
    AnnotationStub stub;
    stub.initialize(visualization);

    // Act 
    PositionVelocity posvel(1, 2, 3, 4, 5, 6, 7, 8);
    visualization->setPosition(posvel);

    // Assert
    ASSERT_EQ(1, stub.anchorPosition.x);
    ASSERT_EQ(2, stub.anchorPosition.y);
    ASSERT_EQ(3, stub.anchorPosition.z);
    ASSERT_EQ(4, stub.anchorPosition.phi);
    ASSERT_EQ(5, stub.anchorPosition.vx);
    ASSERT_EQ(6, stub.anchorPosition.vy);
    ASSERT_EQ(7, stub.anchorPosition.vz);
    ASSERT_EQ(8, stub.anchorPosition.vphi);
}

TEST(Visualization, VisibilityOn_signalVisibilityChangedCalled)
{
    // Arrange
    vtkSmartPointer<MyVisualization> visualization = vtkSmartPointer<MyVisualization>::New();
    AnnotationStub stub;
    stub.initialize(visualization);

    // Act 
    visualization->VisibilityOn();

    // Assert
    ASSERT_TRUE(stub.onAnchorVisibilityChangedCalled);
}

TEST(Visualization, VisibilityOn_signalVisibilityChangedCalledVisibilityTrue)
{
    // Arrange
    vtkSmartPointer<MyVisualization> visualization = vtkSmartPointer<MyVisualization>::New();
    AnnotationStub stub;
    stub.initialize(visualization);

    // Act 
    visualization->VisibilityOn();

    // Assert
    ASSERT_TRUE(stub.anchorVisibility);
}

/*
 * Main entry
 */
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
