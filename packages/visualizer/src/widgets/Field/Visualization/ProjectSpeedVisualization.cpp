 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Tim Kouters, July 2017
 */

#include <vtkAlgorithm.h>
#include <vtkAlgorithmOutput.h>
#include <vtkCubeSource.h>
#include <vtkArrowSource.h>
#include <vtkTransform.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolygon.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkObjectFactory.h>

// Internal:
#include "int/widgets/Field/Visualization/projectSpeedVisualization.h"
#include "int/ConfigurationManager.h"

// Falcons shared code:
#include "tracing.hpp"
#include "linepoint2D.hpp"

projectSpeedVisualization::projectSpeedVisualization() : Visualization()
{
	// Hide
	this->VisibilityOff();
    _arrow->VisibilityOff();

    _lineSource = vtkSmartPointer<vtkLineSource>::New();
	_lineSource->SetPoint1(0.0, 0.0, 0.1);
	_lineSource->SetPoint2(0.0, 0.0, 0.1);
	_lineSource->Update();

    _actor = addAsActor(_lineSource);
    _actor->GetProperty()->SetColor(0.3, 0.0, 1.0);
    _actor->GetProperty()->SetLineWidth(6);
    _actor->SetPosition(0.0, 0.0, 0.0);
    _actor->Modified();
    _actor->VisibilityOff();
}


void projectSpeedVisualization::setPosition(linepoint2D& line)
{
	_actor->SetOrigin(line.getSourcePoint2D().x, line.getSourcePoint2D().y, 0.1);
	Vector2D dst = line.getDestinationVector2D() - line.getSourceVector2D();
	_lineSource->SetPoint1(dst.x, dst.y, 0.1);

	this->VisibilityOn();
	_actor->VisibilityOn();

    // call base implementation
    Visualization::setPosition(line);
}

