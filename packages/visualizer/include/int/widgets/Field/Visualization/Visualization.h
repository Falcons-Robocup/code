// Copyright 2016-2017 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
#include <vtkLineSource.h>

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
        _mapper->SetInputData(o);
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
    vtkSmartPointer<vtkLineSource> _lineSource;

protected:
    PositionVelocity _posvel;
    polygon2D _area;
    linepoint2D _line;
};

#endif // VISUALIZATION_H
