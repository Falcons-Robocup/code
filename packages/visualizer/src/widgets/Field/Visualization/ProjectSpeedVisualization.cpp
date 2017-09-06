 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
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
#include "tracer.hpp"
#include "linepoint2D.hpp"

projectSpeedVisualization::projectSpeedVisualization() : Visualization()
{
	this->VisibilityOff();

	// Setup arrow
	_arrow->GetProperty()->SetColor(0.3, 0.0, 1.0); //Rainbow purple

    _arrow->VisibilityOff();
    this->VisibilityOff();
}


void projectSpeedVisualization::setPosition(linepoint2D& line)
{
	this->VisibilityOff();
	_arrow->VisibilityOff();

	Vector2D speed = line.getSourceVector2D() - line.getDestinationVector2D();
    double length = speed.size();
    double orientation = atan2(speed.y, speed.x);
    _arrow->SetOrientation(0, 0, 180.0 * (orientation + M_PI) / M_PI); // radians to degrees
    if (length > 0.1)
    {
        double scale[3] = {length, 1, 1 };
        _arrow->SetScale(scale);
        _arrowSource->SetTipLength(length);
        _arrow->VisibilityOn();
        this->VisibilityOn();
    }
    else
    {
        _arrow->VisibilityOff();
        this->VisibilityOff();
    }

    // call base implementation
    Visualization::setPosition(line);
}

