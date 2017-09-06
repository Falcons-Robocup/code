 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Visualization.h
 *
 *  Created on: November 20th, 2016
 *      Author: Diana Koenraadt
 */

#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <QObject>
#include <vtkRenderer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtkFollower.h>
#include <vector>
#include <vtkAssembly.h>
#include <vtkTransform.h>
#include <vtkArrowSource.h>
#include <vtkCubeSource.h>

// Internal:
#include "int/types/PositionVelocity.h"

// External:
#include "polygon2D.hpp"
#include "linepoint2D.hpp"

class FieldWidget3D;

/*
* Base class for actors in the field
*/
class Visualization : public QObject, public vtkAssembly // vtkAssembly groups vtkProp3Ds (e.g. vtkActor) together.
{
        Q_OBJECT

public:
    Visualization();
    virtual ~Visualization() {}
    void setPosition(PositionVelocity& posvel);
    void setPosition(polygon2D& area);
    void setPosition(linepoint2D& line);
    PositionVelocity getPosition();

    virtual void VisibilityOn() override;
    virtual void VisibilityOff() override;

Q_SIGNALS:
    void signalPositionChanged(PositionVelocity& posvel);
    void signalPositionChanged(polygon2D& area);
    void signalPositionChanged(linepoint2D& line);
    void signalVisibilityChanged(bool visible);

protected:
    template <typename T>
    vtkSmartPointer<vtkActor> addAsActor(T const &o)
    {
        vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputConnection(o->GetOutputPort());
        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(mapper);
        this->AddPart(actor);
        return actor;
    }

    template <typename T>
    vtkSmartPointer<vtkActor> addAsPolyActor(T const &o)
    {
        _mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        _mapper->SetInput(o);
        vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
        actor->SetMapper(_mapper);
        this->AddPart(actor);
        return actor;
    }

    vtkSmartPointer<vtkArrowSource> _arrowSource;
    vtkSmartPointer<vtkActor> _arrow;
    vtkSmartPointer<vtkPolyData> _polySource;
    vtkSmartPointer<vtkPoints> _points;
    vtkSmartPointer<vtkPolygon> _polygon;
    vtkSmartPointer<vtkCellArray> _polygons;
    vtkSmartPointer<vtkActor> _actor;
    vtkSmartPointer<vtkPolyDataMapper> _mapper;

protected:
    PositionVelocity _posvel;
    polygon2D _area;
    linepoint2D _line;
};

#endif // VISUALIZATION_H
