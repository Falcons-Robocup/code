// Copyright 2019-2020 lucas (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Lucas Catabriga, September 2019
 */

#include <vtkAlgorithm.h>
#include <vtkAlgorithmOutput.h>
#include <vtkSphereSource.h>
#include <vtkRegularPolygonSource.h>
#include <vtkArrowSource.h>
#include <vtkTransform.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkObjectFactory.h>
#include <math.h>

// Internal:
#include "int/widgets/Field/Visualization/GaussianVisualization.h"
#include "int/ConfigurationManager.h"

// Falcons shared code:
#include "tracing.hpp"
#include "vector2d.hpp"
#include "matrix22.hpp"
#include "vector3d.hpp"
#include "matrix33.hpp"

// To understand read this:
// https://www.visiondummy.com/2014/04/geometric-interpretation-covariance-matrix/

GaussianVisualization::GaussianVisualization()
{
           
    vtkSmartPointer<vtkRegularPolygonSource> circle_fill_0 = vtkSmartPointer<vtkRegularPolygonSource>::New();
    circle_fill_0->SetNumberOfSides(50);
    circle_fill_0->SetRadius(1.0);
    circle_fill_0->SetCenter(0, 0, 0);
    fill_circle_actor[0] = addAsActor(circle_fill_0);
    fill_circle_actor[0]->GetProperty()->SetColor(0.0, 0.0, 1.0);

    vtkSmartPointer<vtkRegularPolygonSource> circle_border_0 = vtkSmartPointer<vtkRegularPolygonSource>::New();
    circle_border_0->SetNumberOfSides(50);
    circle_border_0->SetRadius(1.0);
    circle_border_0->SetCenter(0, 0, 0);
    circle_border_0->GeneratePolygonOff();
    border_circle_actor[0] = addAsActor(circle_border_0);
    border_circle_actor[0]->GetProperty()->SetColor(0.0, 0.0, 0.0);    

    vtkSmartPointer<vtkRegularPolygonSource> circle_fill_1 = vtkSmartPointer<vtkRegularPolygonSource>::New();
    circle_fill_1->SetNumberOfSides(50);
    circle_fill_1->SetRadius(1.0);
    circle_fill_1->SetCenter(0, 0, 0);
    circle_fill_1->SetNormal(0, 1, 0);
    fill_circle_actor[1] = addAsActor(circle_fill_1);
    fill_circle_actor[1]->GetProperty()->SetColor(0.0, 0.0, 1.0);

    vtkSmartPointer<vtkRegularPolygonSource> circle_border_1 = vtkSmartPointer<vtkRegularPolygonSource>::New();
    circle_border_1->SetNumberOfSides(50);
    circle_border_1->SetRadius(1.0);
    circle_border_1->SetCenter(0, 0, 0);
    circle_border_1->SetNormal(0, 1, 0);
    circle_border_1->GeneratePolygonOff();
    border_circle_actor[1] = addAsActor(circle_border_1);
    border_circle_actor[1]->GetProperty()->SetColor(0.0, 0.0, 0.0);   

    vtkSmartPointer<vtkRegularPolygonSource> circle_fill_2 = vtkSmartPointer<vtkRegularPolygonSource>::New();
    circle_fill_2->SetNumberOfSides(50);
    circle_fill_2->SetRadius(1.0);
    circle_fill_2->SetCenter(0, 0, 0);
    circle_fill_2->SetNormal(1, 0, 0);
    fill_circle_actor[2] = addAsActor(circle_fill_2);
    fill_circle_actor[2]->GetProperty()->SetColor(0.0, 0.0, 1.0);

    vtkSmartPointer<vtkRegularPolygonSource> circle_border_2 = vtkSmartPointer<vtkRegularPolygonSource>::New();
    circle_border_2->SetNumberOfSides(50);
    circle_border_2->SetRadius(1.0);
    circle_border_2->SetCenter(0, 0, 0);
    circle_border_2->SetNormal(1, 0, 0);
    circle_border_2->GeneratePolygonOff();
    border_circle_actor[2] = addAsActor(circle_border_2);
    border_circle_actor[2]->GetProperty()->SetColor(0.0, 0.0, 0.0);    

    // Create pointing arrow
    _arrow->GetProperty()->SetColor(0.0, 0.0, 0.0); // uniform black coloring

    // Create an assembly consisting of the various parts
    this->VisibilityOn();
}

static Matrix22 getMatrix22FromDiagMatrix22(diagMatrix22 dm)
{
    return Matrix22(dm.matrix[0][0], dm.matrix[0][1], dm.matrix[1][0], dm.matrix[1][1]);
}

static Matrix33 getMatrix33FromDiagMatrix33(diagMatrix33 dm)
{
    Matrix33 m;

    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            m.matrix[i][j] = dm.matrix[i][j];
        }
    }

    return m;
}

static double radToDeg(double rad)
{
    return rad * 180.0 / M_PI;
}

void GaussianVisualization::setColor(double r, double g, double b)
{
    for(int i=0; i<3; i++)
    {
        border_circle_actor[i]->GetProperty()->SetColor(r, g, b);
        fill_circle_actor[i]->GetProperty()->SetColor(r, g, b);
    }
}

void GaussianVisualization::setValue(const diagGaussian2D& gaussian_2d)
{
    Matrix22 covariance = getMatrix22FromDiagMatrix22(gaussian_2d.covariance);

    Vector2D eingenValues = covariance.getEigenValues();
    Matrix22 eingenVectors = covariance.getEigenVectors(eingenValues);

    // To draw the ellipse the eingen vector corresponding to the largest eingen value needs to be found, it will be stored on V1
    // L1 will hold the largest eingen value, L2 the smallest eingen value
    double L1;
    double L2;
    Vector2D V1;

    if(eingenValues[0] > eingenValues[1])
    {
        L1 = eingenValues[0];
        L2 = eingenValues[1];
        V1 = Vector2D(eingenVectors.matrix[0][0], eingenVectors.matrix[1][0]);
    }
    else
    {
        L1 = eingenValues[1];
        L2 = eingenValues[0];
        V1 = Vector2D(eingenVectors.matrix[0][1], eingenVectors.matrix[1][1]);
    }

    double chi_95 = 5.991146;
    double width = sqrt(L1 * chi_95);
    double height = sqrt(L2 * chi_95);
    double angle = radToDeg(atan2(V1[1], V1[0]));

    fill_circle_actor[0]->SetScale(width, height, 1.0);
    border_circle_actor[0]->SetScale(width, height, 1.0);

    fill_circle_actor[0]->SetOrientation(0, 0, angle);
    border_circle_actor[0]->SetOrientation(0, 0, angle);

    
    double opacity = std::max(0.0, 0.2 - (width + height)/50.0);
    fill_circle_actor[0]->GetProperty()->SetOpacity(opacity);
    border_circle_actor[0]->GetProperty()->SetOpacity(5.0 * opacity);
    
    
    for(int i=1; i<3; i++)
    {
        // For 2D data only first gaussian is necessary
        fill_circle_actor[i]->VisibilityOff();
        border_circle_actor[i]->VisibilityOff();
    }

    _arrow->VisibilityOff();

    PositionVelocity posvel(gaussian_2d.mean.x, gaussian_2d.mean.y);
    Visualization::setPosition(posvel);
}

static Matrix33 normalizeEigenVectors(Matrix33 eigenVectors)
{
    Matrix33 normEigVec;

    for(int j=0; j<3; j++)
    {
        double norm = sqrt(eigenVectors.matrix[0][j]*eigenVectors.matrix[0][j] +
                           eigenVectors.matrix[1][j]*eigenVectors.matrix[1][j] +
                           eigenVectors.matrix[2][j]*eigenVectors.matrix[2][j]);

        for(int i=0; i<3; i++)
        {
            normEigVec.matrix[i][j] = eigenVectors.matrix[i][j] / norm;
        }
    }  

    return normEigVec;
}

// static void printMatrix(Matrix33 m)
// {
//     printf("[");
//     for(int i=0; i<3; i++)
//     {
//         printf("[%f, %f, %f]", m.matrix[i][0],m.matrix[i][1],m.matrix[i][2]);
//         if(i!=2)
//         {
//             printf(",");
//         }
//     }
//     printf("]\n");
// }

static Vector3D getAnglesFromRotationMatrix(Matrix33 eigenVectors, Vector3D eigenValues)
{
    // VTK rotation order is Z-X-Y (https://vtk.org/doc/nightly/html/classvtkProp3D.html#a1c44f66f6ce311d9f38b9e1223f9cee5)
    // so reversing the rotation matrix RyRxRz is necessary (https://www.geometrictools.com/Documentation/EulerAngles.pdf)

    // Matrix33 rotMat = normalizeEigenVectors(eigenVectors);

    // Vector3D rotations;

    // if(rotMat.matrix[2][1] < 1.0)
    // {
    //     if(rotMat.matrix[2][1] > -1.0)
    //     {
    //         rotations[0] = asin(rotMat.matrix[2][1]);
    //         rotations[1] = atan2(-rotMat.matrix[2][0],rotMat.matrix[2][2]);
    //         rotations[2] = atan2(-rotMat.matrix[0][1],rotMat.matrix[1][1]);
    //     }
    //     else
    //     {
    //         rotations[0] = -M_PI/2.0;
    //         rotations[1] = 0.0;
    //         rotations[2] = -atan2(rotMat.matrix[0][2],rotMat.matrix[0][0]);
    //     }
    // }
    // else
    // {
    //     rotations[0] = M_PI/2.0;
    //     rotations[1] = 0.0;
    //     rotations[2] = atan2(rotMat.matrix[0][2],rotMat.matrix[0][0]);
    // }

    // TODO
    // The previous method does not work, I don't understand why
    // This next implementation works well, but is incorrectly setting one of 
    // the angles to zero. But because this angle is X, setting it ruins the 
    // next rotation.

    Vector3D rotations;

    Matrix33 rotMat = normalizeEigenVectors(eigenVectors);
    rotations[0] = 0.0;
    rotations[1] = atan2(-rotMat.matrix[2][0],sqrt(rotMat.matrix[1][0]*rotMat.matrix[1][0] + rotMat.matrix[0][0]*rotMat.matrix[0][0]));
    rotations[2] = atan2(rotMat.matrix[1][0],rotMat.matrix[0][0]);
    

    // printf("getAnglesFromRotationMatrix\n");
    // printf("[%f %f %f]\n", eigenValues[0], eigenValues[1], eigenValues[2]);
    // printMatrix(eigenVectors);
    // printMatrix(rotMat);
    // printf("[%f %f %f]\n", rotations[0], rotations[1], rotations[2]);

    return rotations;
}


void GaussianVisualization::setValue(const diagGaussian3D& gaussian_3d)
{
    Matrix33 covariance = getMatrix33FromDiagMatrix33(gaussian_3d.covariance);

    Vector3D eigenValues = covariance.getEigenValues();
    Matrix33 eigenVectors = covariance.getEigenVectors();

    double chi_95 = 5.991146;
    double s0 = sqrt(eigenValues[0] * chi_95);
    double s1 = sqrt(eigenValues[1] * chi_95);
    double s2 = sqrt(eigenValues[2] * chi_95);    
    Vector3D angles = getAnglesFromRotationMatrix(eigenVectors, eigenValues);
    double opacity = std::max(0.05, 0.2 - (s0 + s1 + s2)/50.0);

    fill_circle_actor[0]->SetScale(s0, s1, 1.0);
    border_circle_actor[0]->SetScale(s0, s1, 1.0);
    fill_circle_actor[1]->SetScale(s0, 1.0, s2);
    border_circle_actor[1]->SetScale(s0, 1.0, s2);
    fill_circle_actor[2]->SetScale(1.0, s1, s2);
    border_circle_actor[2]->SetScale(1.0, s1, s2);

    for(int i=0; i<3; i++)
    {
        fill_circle_actor[i]->SetOrientation(radToDeg(angles[0]), radToDeg(angles[1]), radToDeg(angles[2]));
        border_circle_actor[i]->SetOrientation(radToDeg(angles[0]), radToDeg(angles[1]), radToDeg(angles[2]));
        
        fill_circle_actor[i]->GetProperty()->SetOpacity(opacity);
        border_circle_actor[i]->GetProperty()->SetOpacity(5.0 * opacity);

        fill_circle_actor[i]->VisibilityOn();
        border_circle_actor[i]->VisibilityOn();
    }

    _arrow->VisibilityOff();

    PositionVelocity posvel(gaussian_3d.mean.x, gaussian_3d.mean.y, gaussian_3d.mean.z);
    Visualization::setPosition(posvel);
}

