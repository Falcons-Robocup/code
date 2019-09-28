 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * RobotLabel.h
 *
 *  Created on: November 26, 2016
 *      Author: Diana Koenraadt
 */

#include <vtkVectorText.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <math.h> // sin, cos
#include <vtkPoints.h>
#include <vtkPolygon.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <stdlib.h> // rand

// Internal:
#include "int/widgets/Field/Annotation/CollisionBalloon.h"

void CollisionBalloon::initialize(RobotVisualization* anchor, CollisionBalloonDimensions dim) 
{
    Annotation::initialize(anchor);

    addBalloon(dim);

    // Set position only once.
    PositionVelocity posvel = anchor->getPosition();
    this->SetPosition(posvel.x, posvel.y, posvel.z);
}

void CollisionBalloon::setColor(double red, double green, double blue)
{
    _actor->GetProperty()->SetColor(red, green, blue);
}

void CollisionBalloon::onAnchorVisibilityChanged(bool visible)
{
    this->SetVisibility(visible);
};

void CollisionBalloon::addBalloon(CollisionBalloonDimensions dim)
{
    vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();

    vtkSmartPointer<vtkPoints> innerPoints = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkPoints> outerPoints = vtkSmartPointer<vtkPoints>::New();
    for (double i = 0; i < 360; i += 20)
    {
        double x = dim.InnerRadiusX * cos(i * M_PI / 180);
        double y = dim.InnerRadiusY * sin(i * M_PI / 180);
        innerPoints->InsertNextPoint(x, y, 0);

        x = dim.OuterRadiusX * cos(i * M_PI / 180);
        y = dim.OuterRadiusY * sin(i * M_PI / 180);
        outerPoints->InsertNextPoint(x, y, 0);
    }
    vtkSmartPointer<vtkPolygon> innerPolygon = vtkSmartPointer<vtkPolygon>::New();
    innerPolygon->GetPointIds()->SetNumberOfIds(innerPoints->GetNumberOfPoints());
    for (int i = 0; i < innerPoints->GetNumberOfPoints(); i++)
    {
        innerPolygon->GetPointIds()->SetId(i /* index */, i /* point id */); // Iterate over points in the order they were created.
    }
    polygons->InsertNextCell(innerPolygon);

    for (int i = 0; i < innerPoints->GetNumberOfPoints(); ++i)
    {
        vtkSmartPointer<vtkPolygon> triangle = vtkSmartPointer<vtkPolygon>::New();
        triangle->GetPointIds()->SetNumberOfIds(3);

        if (i == innerPoints->GetNumberOfPoints() - 1)
        {
            // Last triangle should connect first and last points
            triangle->GetPointIds()->SetId(0, i);
            triangle->GetPointIds()->SetId(1, innerPoints->GetNumberOfPoints() + i); // outer points will be appended to innerPoints, count onwards
            triangle->GetPointIds()->SetId(2, 0);
        }
        else
        {
            triangle->GetPointIds()->SetId(0, i);
            triangle->GetPointIds()->SetId(1, innerPoints->GetNumberOfPoints() + i); // outer points will be appended to innerPoints, count onwards
            triangle->GetPointIds()->SetId(2, i + 1);
        }

        polygons->InsertNextCell(triangle);
    }

    for (int i = 0; i < outerPoints->GetNumberOfPoints(); ++i)
    {
        innerPoints->InsertNextPoint(outerPoints->GetPoint(i));
    }
    
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(innerPoints);
    polyData->SetPolys(polygons);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    _actor = vtkSmartPointer<vtkActor>::New();
    _actor->SetMapper(mapper);
    _actor->GetProperty()->SetColor(1, 0, 0);

    this->AddPart(_actor);
}
