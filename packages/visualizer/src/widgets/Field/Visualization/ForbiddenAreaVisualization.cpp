 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Tim Kouters, April 2017
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
#include "int/widgets/Field/Visualization/ForbiddenAreaVisualization.h"
#include "int/ConfigurationManager.h"

// Falcons shared code:
#include "tracing.hpp"
#include "vector2d.hpp"

ForbiddenAreaVisualization::ForbiddenAreaVisualization()
{
	this->VisibilityOff();

	// Setup four points
	_points =	vtkSmartPointer<vtkPoints>::New();
	std::vector<Point2D> points = _area.getPoints();
	for(auto it = points.begin(); it != points.end(); it++)
	{
		_points->InsertNextPoint(it->x, it->y, 0.1);
	}

	// Set settings of polygon
	_polygon = vtkSmartPointer<vtkPolygon>::New();
	_polygon->GetPointIds()->SetNumberOfIds(_area.getPoints().size());
	for(size_t i = 0; i < _area.getPoints().size(); i++)
	{
		_polygon->GetPointIds()->SetId(i, i);
	}

	// Add the polygon to a list of polygons
	_polygons = vtkSmartPointer<vtkCellArray>::New();
	_polygons->InsertNextCell(_polygon);

	// Create a PolyData
	_polySource = vtkSmartPointer<vtkPolyData>::New();
	_polySource->SetPoints(_points);
	_polySource->SetPolys(_polygons);

	_actor = addAsPolyActor(_polySource);
    _actor->GetProperty()->SetColor(0.537, 0.812, 0.941); // blue-ish color for forbidden areas
    _actor->GetProperty()->SetOpacity(0.3);
    
    // Create pointing arrow
    _arrow->GetProperty()->SetColor(0.0, 0.0, 0.0); // uniform black coloring

    // Create an assembly consisting of the various parts
    _arrow->VisibilityOff();
    this->VisibilityOff();
}


void ForbiddenAreaVisualization::setPosition(polygon2D& area)
{
	this->VisibilityOff();

	_points->Reset();
	_polygon->GetPoints()->Reset();
	_polygon->GetPointIds()->Reset();
	_polygons->Reset();
	_polySource->GetPoints()->Reset();
	_polySource->GetPolys()->Reset();


	// Setup four points
	//_points = vtkSmartPointer<vtkPoints>::New();
	std::vector<Point2D> points = area.getPoints();
	for(auto it = points.begin(); it != points.end(); it++)
	{
		_points->InsertNextPoint(it->x, it->y, _OBSTACLE_DIAMETER / 3.0);
	}

	// Set settings of polygon
	//_polygon = vtkSmartPointer<vtkPolygon>::New();
	_polygon->GetPointIds()->SetNumberOfIds((vtkIdType)area.getPoints().size());
	for(size_t i = 0; i < area.getPoints().size(); i++)
	{
		_polygon->GetPointIds()->SetId((vtkIdType)i, (vtkIdType)i);
	}

	// Add the polygon to a list of polygons
	//_polygons = vtkSmartPointer<vtkCellArray>::New();
	_polygons->InsertNextCell(_polygon);

	// Create a PolyData
	_polySource->SetPoints(_points);
	_polySource->SetPolys(_polygons);
	_mapper->SetInputData(_polySource);
	_actor->SetMapper(_mapper);
	this->AddPart(_actor);
	_points->Modified();
	_polygon->Modified();
	_polygons->Modified();
	_polySource->Modified();
	_mapper->Modified();
	_actor->Modified();
	_actor->GetMapper()->GetInput()->Modified();
	this->VisibilityOn();
	_arrow->VisibilityOff();

    // call base implementation
    Visualization::setPosition(area);
}

