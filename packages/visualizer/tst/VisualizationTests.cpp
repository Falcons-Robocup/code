 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
