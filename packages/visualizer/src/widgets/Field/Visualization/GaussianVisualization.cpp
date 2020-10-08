 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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

GaussianVisualization::GaussianVisualization()
{
    vtkSmartPointer<vtkRegularPolygonSource> circle_fill = vtkSmartPointer<vtkRegularPolygonSource>::New();
    circle_fill->SetNumberOfSides(50);
    circle_fill->SetRadius(1.0);
    circle_fill->SetCenter(0, 0, 0);
    fill_circle_actor = addAsActor(circle_fill);
    fill_circle_actor->GetProperty()->SetColor(0.0, 0.0, 1.0);
    

    vtkSmartPointer<vtkRegularPolygonSource> circle_border = vtkSmartPointer<vtkRegularPolygonSource>::New();
    circle_border->SetNumberOfSides(50);
    circle_border->SetRadius(1.0);
    circle_border->SetCenter(0, 0, 0);
    circle_border->GeneratePolygonOff();
    border_circle_actor = addAsActor(circle_border);
    border_circle_actor->GetProperty()->SetColor(0.0, 0.0, 0.0);    
    
    // Create pointing arrow
    _arrow->GetProperty()->SetColor(0.0, 0.0, 0.0); // uniform black coloring

    // Create an assembly consisting of the various parts
    this->VisibilityOn();
}

static Matrix22 getMatrix22FromDiagMatrix22(diagMatrix22 dm)
{
    return Matrix22(dm.matrix[0][0], dm.matrix[0][1], dm.matrix[1][0], dm.matrix[1][1]);
}

static double radToDeg(double rad)
{
    return rad * 180.0 / M_PI;
}

void GaussianVisualization::setColor(double r, double g, double b)
{
    border_circle_actor->GetProperty()->SetColor(r, g, b);
    fill_circle_actor->GetProperty()->SetColor(r, g, b);
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

    fill_circle_actor->SetScale(width, height, 1.0);
    border_circle_actor->SetScale(width, height, 1.0);

    fill_circle_actor->SetOrientation(0, 0, angle);
    border_circle_actor->SetOrientation(0, 0, angle);

    
    double opacity = std::max(0.0, 0.2 - (width + height)/50.0);
    fill_circle_actor->GetProperty()->SetOpacity(opacity);
    border_circle_actor->GetProperty()->SetOpacity(5.0 * opacity);
    

    _arrow->VisibilityOff();

    PositionVelocity posvel(gaussian_2d.mean.x, gaussian_2d.mean.y);
    Visualization::setPosition(posvel);
}

