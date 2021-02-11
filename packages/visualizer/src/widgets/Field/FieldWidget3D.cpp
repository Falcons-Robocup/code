// Copyright 2016-2020 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include <vtkAlgorithm.h>
#include <vtkAlgorithmOutput.h>
#include <vtkCellArray.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolygon.h>
#include <vtkSphereSource.h>
#include <vtkTextActor.h>

// Internal:
#include "int/widgets/Field/FieldWidget3D.h"
#include "int/widgets/Field/FieldVideoActor.h"
#include "int/widgets/Field/Annotation/RobotLabel.h"
#include "int/widgets/Field/Annotation/CollisionBalloon.h"
#include "int/ConfigurationManager.h"

#include <boost/format.hpp>

// Falcons shared code:
#include "tracing.hpp"
#include "vector3d.hpp"
#include "cEnvironmentField.hpp" // field dimensions

/*
* ========================================
*           constructor/destructor
* ========================================
*/

FieldWidget3D::FieldWidget3D(QWidget *parent)
  : QVTKWidget(parent),
    Widget()
{
    TRACE(">");
    // Set up rendering
    _renderWindow = vtkRenderWindow::New();
    _mainRenderer = vtkRenderer::New();
    _mainRenderer->SetBackground(72.0 / 255.0, 72.0 / 255.0, 72.0 / 255.0);
    _mainRenderer->SetLayer(0);

    _annotationRenderer = vtkRenderer::New();
    _annotationRenderer->SetLayer(1);

    _renderWindow->SetNumberOfLayers(2);
    _renderWindow->AddRenderer(_mainRenderer);
    _renderWindow->AddRenderer(_annotationRenderer);
    this->SetRenderWindow(_renderWindow);

    // Interactor
    QVTKInteractor* iren = this->GetInteractor();
    _renderWindow->SetInteractor(iren);

    // Camera properties
    _camera = vtkCamera::New();
    _mainRenderer->SetActiveCamera(_camera);
    _annotationRenderer->SetActiveCamera(_camera);

    addFieldActors();
    addGoalActors();

    for (int i = 1; i <= _NR_OF_ROBOTS_PER_TEAM; ++i)
    {
        addRobotActor(i);
    }

    // Create a few ball actors
    for (int i = 1; i <= 3 * _NR_OF_ROBOTS_PER_TEAM; ++i)
    {
        addBallActor();
    }

    // Create quite some obstacle actors
    for (int i = 1; i <= 20 * _NR_OF_ROBOTS_PER_TEAM; ++i)
    {
        addObstacleActor();
        addForbiddenAreaActor();
        addProjectSpeedActor();
    }

    for (int i = 1; i <= _NR_OF_ROBOTS_PER_TEAM; ++i)
    {
        addShootTargetActor(i);
    }

    addTechnicalTeamAreaActor();

    _trueBallActor = vtkSmartPointer<BallVisualization>::New();
    _mainRenderer->AddActor(_trueBallActor);
    _trueBallActor->VisibilityOff();

    resetZoomPanRotate();

    // Start timers
    _updateTimer = new QTimer();
    connect(_updateTimer, SIGNAL(timeout()), this, SLOT(render()));
    _updateTimer->start(100);
    TRACE("<");
}

FieldWidget3D::~FieldWidget3D()
{
    // Delete actors
    for (int i = 1; i <= _NR_OF_ROBOTS_PER_TEAM; ++i)
    {
        _mainRenderer->RemoveActor(_teamActors[i]);
    }
    _teamActors.clear();

    for (size_t i = 0; i < _obstacleActors.size(); i++)
    {
        _mainRenderer->RemoveActor(_obstacleActors[i]);
    }
    _obstacleActors.clear();

    for (size_t i = 0; i < _forbiddenAreaActors.size(); i++)
    {
        _mainRenderer->RemoveActor(_forbiddenAreaActors[i]);
    }
    _forbiddenAreaActors.clear();

    for (size_t i = 0; i < _ballActors.size(); i++)
    {
        _mainRenderer->RemoveActor(_ballActors[i]);
    }
    _ballActors.clear();

    for (size_t i = 0; i < _projectSpeedActors.size(); i++)
    {
        _mainRenderer->RemoveActor(_projectSpeedActors[i]);
    }
    _projectSpeedActors.clear();

    for (size_t i = 0; i < _gaussianObstacleActors.size(); i++)
    {
        _mainRenderer->RemoveActor(_gaussianObstacleActors[i]);
    }
    _gaussianObstacleActors.clear();

    _ballActors.clear();

    _mainRenderer->RemoveActor(_trueBallActor);

    for (size_t i = 0; i < _eventHandlers.size(); i++)
    {
        delete _eventHandlers[i];
    }
    _eventHandlers.clear();
}

void FieldWidget3D::saveState()
{
    QSettings settings;

    settings.setValue("fieldWidget", 1); // QSettings versioning, increase if you rename fieldWidget keys.

    // Save camera position
    vtkCamera* camera = _mainRenderer->GetActiveCamera();
    double d[4];
    camera->GetPosition(d);
    settings.setValue("fieldWidget/cameraPosition/x", d[0]);
    settings.setValue("fieldWidget/cameraPosition/y", d[1]);
    settings.setValue("fieldWidget/cameraPosition/z", d[2]);

    camera->GetFocalPoint(d);
    settings.setValue("fieldWidget/cameraFocalPoint/x", d[0]);
    settings.setValue("fieldWidget/cameraFocalPoint/y", d[1]);
    settings.setValue("fieldWidget/cameraFocalPoint/z", d[2]);

    camera->GetViewUp(d);
    settings.setValue("fieldWidget/cameraViewUp/x", d[0]);
    settings.setValue("fieldWidget/cameraViewUp/y", d[1]);
    settings.setValue("fieldWidget/cameraViewUp/z", d[2]);

    settings.sync();
}

void FieldWidget3D::restoreState()
{
    QSettings settings;
    if (settings.contains("fieldWidget") && settings.value("fieldWidget").toInt() == 1)
    {
        // Restore camera position
        resetZoomPanRotate();

        vtkCamera* camera = _mainRenderer->GetActiveCamera();
        camera->SetPosition(settings.value("fieldwidget/cameraPosition/x").toDouble(),
                            settings.value("fieldwidget/cameraPosition/y").toDouble(),
                            settings.value("fieldwidget/cameraPosition/z").toDouble());
        camera->SetFocalPoint(settings.value("fieldwidget/cameraFocalPoint/x").toDouble(),
                              settings.value("fieldwidget/cameraFocalPoint/y").toDouble(),
                              settings.value("fieldwidget/cameraFocalPoint/z").toDouble());
        camera->SetViewUp(settings.value("fieldwidget/cameraViewUp/x").toDouble(),
                          settings.value("fieldwidget/cameraViewUp/y").toDouble(),
                          settings.value("fieldwidget/cameraViewUp/z").toDouble());
    }
}

/*
* ========================================
*           protected
* ========================================
*/


void FieldWidget3D::resizeEvent(QResizeEvent * event)
{
    resetZoomPanRotate();
}

void FieldWidget3D::reloadSettings()
{
    QSettings settings;

    // Enable/disable path planning visualization
    bool showPathPlanning = settings.value(Visualizer::Settings::showPathPlanningSetting, false).toBool();
    for (size_t i = 0; i < _teamActors.size(); i++)
    {
        getTeamRobot((uint8_t)i + 1)->setPathPlanningEnabled(showPathPlanning);
    }
};

/*
* ========================================
*           public Q_SLOTS
* ========================================
*/

void FieldWidget3D::resetZoomPanRotate()
{
    _renderMutex.lock();

    // Undo rotating, zooming and panning
    _mainRenderer->ResetCamera();
    vtkCamera* camera = _mainRenderer->GetActiveCamera();

    // The camera position needs to be calculated based on the maximum bounding boxes for the widget.
    int width = this->width();
    int height = this->height();

    camera->SetPosition(0.0, 0.0, height*45.0/width);

    camera->SetViewUp((_flipField ? 1 : -1), 0, 0);

    _mainRenderer->ResetCameraClippingRange(); // Don't forget this, otherwise you see nothing after resetting...
    _mainRenderer->Modified();
    _annotationRenderer->ResetCameraClippingRange(); // Don't forget this, otherwise you see nothing after resetting...
    _annotationRenderer->Modified();

    _renderMutex.unlock();
}

void FieldWidget3D::flip(bool flip)
{
    _flipField = flip;

    fieldVideoActor->setFieldFlip(flip);

    resetZoomPanRotate();
}

void FieldWidget3D::switchHeightmapToNone(void)
{
    _heightmapName = CompositeHeightmapName::INVALID;
}

void FieldWidget3D::switchHeightmapToDefendAttackingOpponent(void)
{
    _heightmapName = CompositeHeightmapName::DEFEND_ATTACKING_OPPONENT;
}

void FieldWidget3D::switchHeightmapToDribble(void)
{
    _heightmapName = CompositeHeightmapName::DRIBBLE;
}

void FieldWidget3D::switchHeightmapToMoveToFreeSpot(void)
{
    _heightmapName = CompositeHeightmapName::MOVE_TO_FREE_SPOT;
}

void FieldWidget3D::switchHeightmapToPositionForOppSetpiece(void)
{
    _heightmapName = CompositeHeightmapName::POSITION_FOR_OPP_SETPIECE;
}

void FieldWidget3D::switchHeightmapToPositionForOwnSetpiece(void)
{
    _heightmapName = CompositeHeightmapName::POSITION_FOR_OWN_SETPIECE;
}

void FieldWidget3D::switchHeightmapForRobot(int robotId)
{
    _robotID = robotId;
}

/*
* ========================================
*           private Q_SLOTS
* ========================================
*/

void FieldWidget3D::render()
{
    _renderMutex.lock();

    _mainRenderer->ResetCameraClippingRange(); // Recalculates the near and far clipping planes.
    _mainRenderer->Modified();
    _renderWindow->Render();

    _renderMutex.unlock();

    cleanup();

    WRITE_TRACE;
}

void FieldWidget3D::cleanup()
{
    // time after which actor should be hidden
    double timeout = 0.12; // TODO magic, make configurable?

    // Make obstacle actors invisible that have not moved in the last few render steps
    int countVisibleObstacles = 0;
    for (auto it = _obstacleMapping.begin(); it != _obstacleMapping.end(); /* no increment */ )
    {
        if (_timestamp > it->second.second + timeout)
        {
            _obstacleActors[it->second.first]->VisibilityOff();
            TRACE("hiding obstacle id=%d", it->first);
            // delete from mapping
            it = _obstacleMapping.erase(it);
        }
        else
        {
            ++it;
            ++countVisibleObstacles;
        }

    }
    TRACE("#obstAct=%d #obstMap=%d #obstVis=%d", (int)_obstacleActors.size(), (int)_obstacleMapping.size(), countVisibleObstacles);

    int countVisibleForbiddenAreas = 0;
    for (auto it = _forbiddenAreaMapping.begin(); it != _forbiddenAreaMapping.end(); /* no increment */ )
    {
        if (_timestamp > it->second.second + timeout)
        {
            _forbiddenAreaActors[it->second.first]->VisibilityOff();
            TRACE("hiding obstacle id=%d", it->first);
            // delete from mapping
            it = _forbiddenAreaMapping.erase(it);
        }
        else
        {
            ++it;
            ++countVisibleForbiddenAreas;
        }

    }
    TRACE("#areaAct=%d #areaMap=%d #areaVis=%d", (int)_forbiddenAreaActors.size(), (int)_forbiddenAreaMapping.size(), countVisibleForbiddenAreas);

    // Make ball actors invisible that have not moved in the last few render steps
    int countVisibleBalls = 0;
    for (auto it = _ballMapping.begin(); it != _ballMapping.end(); /* no increment */ )
    {
        if (_timestamp > it->second.second + timeout)
        {
            _ballActors[it->second.first]->VisibilityOff();
            TRACE("hiding ball id=%d", it->first);
            // delete from mapping
            it = _ballMapping.erase(it);
        }
        else
        {
            ++it;
            ++countVisibleBalls;
        }

    }
    TRACE("#ballAct=%d #ballMap=%d #ballVis=%d", (int)_ballActors.size(), (int)_ballMapping.size(), countVisibleBalls);

    int countVisibleProjectSpeeds = 0;
    for (auto it = _projectSpeedMapping.begin(); it != _projectSpeedMapping.end(); /* no increment */ )
    {
        if (_timestamp > it->second.second + timeout)
        {
            _projectSpeedActors[it->second.first]->VisibilityOff();
            TRACE("hiding obstacle id=%d", it->first);
            // delete from mapping
            it = _projectSpeedMapping.erase(it);
        }
        else
        {
            ++it;
            ++countVisibleProjectSpeeds;
        }

    }
    TRACE("#speedAct=%d #speedMap=%d #speedVis=%d", (int)_projectSpeedActors.size(), (int)_projectSpeedMapping.size(), countVisibleProjectSpeeds);
}

/*
* ========================================
*           public
* ========================================
*/

vtkSmartPointer<RobotVisualization> FieldWidget3D::getTeamRobot(uint8_t robotId)
{
    assert(_teamActors.count(robotId));

    return _teamActors[robotId];
}

void FieldWidget3D::setLogTimeStamp(double t)
{
    _timestamp = t;
}

void FieldWidget3D::setClockTick(double elapsedTime, double actualTime)
{
    fieldVideoActor->updateImage(actualTime, _heightmapName, _robotID);
}

/*
* ========================================
*           private
* ========================================
*/

void FieldWidget3D::clear()
{
    for (size_t i = 0; i < _teamActors.size(); i++)
    {
        getTeamRobot((uint8_t)i + 1)->VisibilityOff();
    }

    _obstacleMapping.clear();
    for (size_t i = 0; i < _obstacleActors.size(); i++)
    {
        _obstacleActors[i]->VisibilityOff();
    }

    _forbiddenAreaMapping.clear();
    for (size_t i = 0; i < _forbiddenAreaActors.size(); i++)
    {
        _forbiddenAreaActors[i]->VisibilityOff();
    }

    _ballMapping.clear();
    for (size_t i = 0; i < _ballActors.size(); i++)
    {
        _ballActors[i]->VisibilityOff();
    }

    _projectSpeedMapping.clear();
    for (size_t i = 0; i < _projectSpeedActors.size(); i++)
    {
        _projectSpeedActors[i]->VisibilityOff();
    }

    _technicalTeamAreaActor->VisibilityOff();

    for (size_t i = 0; i < _gaussianObstacleActors.size(); i++)
    {
        _gaussianObstacleActors[i]->VisibilityOff();
    }
}

void FieldWidget3D::addFieldActors()
{
    TRACE("addFieldActors");
    double greenR = 0.278, greenG = 0.64, greenB = 0.196;

    // Get field dimensions
    float _FIELD_LENGTH = cEnvironmentField::getInstance().getLength();
    float _FIELD_WIDTH = cEnvironmentField::getInstance().getWidth();
    poiInfo poi;
    cEnvironmentField::getInstance().getFieldPOI(P_OPP_GOALAREA_CORNER_RIGHT, poi);
    float _GOAL_AREA_LENGTH = 0.5 * _FIELD_LENGTH - poi.y;
    float _GOAL_AREA_WIDTH = poi.x * 2.0;
    cEnvironmentField::getInstance().getFieldPOI(P_OPP_PENALTYAREA_CORNER_RIGHT, poi);
    float _PENALTY_AREA_LENGTH = 0.5 * _FIELD_LENGTH - poi.y;
    float _PENALTY_AREA_WIDTH = poi.x * 2.0;
    cEnvironmentField::getInstance().getFieldPOI(P_CIRCLE_INTERSECT_LINE_RIGHT, poi);
    float _CENTER_CIRCLE_RADIUS = poi.x;
    cEnvironmentField::getInstance().getFieldPOI(P_OPP_PENALTY_SPOT, poi);
    float _PENALTY_MARK_DISTANCE = 0.5 * _FIELD_LENGTH - poi.y;
    TRACE("%30s = %7.3f", "_FIELD_LENGTH", _FIELD_LENGTH);
    TRACE("%30s = %7.3f", "_FIELD_WIDTH", _FIELD_WIDTH);
    TRACE("%30s = %7.3f", "_GOAL_AREA_LENGTH", _GOAL_AREA_LENGTH);
    TRACE("%30s = %7.3f", "_GOAL_AREA_WIDTH", _GOAL_AREA_WIDTH);
    TRACE("%30s = %7.3f", "_PENALTY_AREA_LENGTH", _PENALTY_AREA_LENGTH);
    TRACE("%30s = %7.3f", "_PENALTY_AREA_WIDTH", _PENALTY_AREA_WIDTH);
    TRACE("%30s = %7.3f", "_CENTER_CIRCLE_RADIUS", _CENTER_CIRCLE_RADIUS);
    TRACE("%30s = %7.3f", "_PENALTY_MARK_DISTANCE", _PENALTY_MARK_DISTANCE);

    // Draw plane
    vtkSmartPointer<vtkPlaneSource> planeSrc = vtkSmartPointer<vtkPlaneSource>::New();
    planeSrc->SetOrigin(0, 0, 0);
    planeSrc->SetPoint1(_FIELD_WIDTH + 2.0, 0, 0);
    planeSrc->SetPoint2(0, _FIELD_LENGTH + 2.0, 0);
    planeSrc->Update();

    // vtkSmartPointer<vtkPolyDataMapper> planeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    // planeMapper->SetInputData(planeSrc->GetOutput());
    // this->_field = vtkActor::New();
    // this->_field->SetMapper(planeMapper);
    // this->_field->GetProperty()->SetColor(greenR, greenG, greenB);
    // this->_field->GetProperty()->SetOpacity(1.0);
    // this->_field->SetPosition(-_FIELD_WIDTH / 2 - 1.0, -_FIELD_LENGTH / 2 - 1.0, -0.02);
    // this->_field->GetProperty()->SetAmbient(1);
    // this->_field->GetProperty()->SetDiffuse(0);
    // this->_field->GetProperty()->SetSpecular(0);
    // this->update();
    // _mainRenderer->AddActor(_field);

    fieldVideoActor = vtkSmartPointer<FieldVideoActor>::New();
    _mainRenderer->AddActor(fieldVideoActor);

    // Draw playing direction arrow in a lighter color.
    vtkSmartPointer<vtkPoints> arrowPoints = vtkSmartPointer<vtkPoints>::New();
    double eightFW = _FIELD_WIDTH / 8;
    double eightFL = _FIELD_LENGTH / 8;
    arrowPoints->InsertNextPoint(-eightFW, -eightFL, 0.0);
    arrowPoints->InsertNextPoint(-eightFW, eightFL, 0.0);
    arrowPoints->InsertNextPoint(-2 * eightFW, eightFL, 0.0);
    arrowPoints->InsertNextPoint(0, 2 * eightFL, 0.0);
    arrowPoints->InsertNextPoint(2 * eightFW, eightFL, 0.0);
    arrowPoints->InsertNextPoint(eightFW, eightFL, 0.0);
    arrowPoints->InsertNextPoint(eightFW, -eightFL, 0.0);

    vtkSmartPointer<vtkPolygon> arrowHead = vtkSmartPointer<vtkPolygon>::New();
    arrowHead->GetPointIds()->SetNumberOfIds(3);
    arrowHead->GetPointIds()->SetId(0 /* index */, 2 /* point id */);
    arrowHead->GetPointIds()->SetId(1, 3);
    arrowHead->GetPointIds()->SetId(2, 4);

    vtkSmartPointer<vtkPolygon> arrowBody = vtkSmartPointer<vtkPolygon>::New();
    arrowBody->GetPointIds()->SetNumberOfIds(4);
    arrowBody->GetPointIds()->SetId(0, 0);
    arrowBody->GetPointIds()->SetId(1, 1);
    arrowBody->GetPointIds()->SetId(2, 5);
    arrowBody->GetPointIds()->SetId(3, 6);

    vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();
    polygons->InsertNextCell(arrowHead);
    polygons->InsertNextCell(arrowBody);

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(arrowPoints);
    polyData->SetPolys(polygons);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    vtkSmartPointer<vtkActor> arrowActor = vtkSmartPointer<vtkActor>::New();
    arrowActor->SetMapper(mapper);
    arrowActor->GetProperty()->SetColor(greenR + 0.2, greenG + 0.2, greenB + 0.2); // Lighter green
    arrowActor->GetProperty()->SetOpacity(0.5);

    _mainRenderer->AddActor(arrowActor);

    // Draw Field
    _mainRenderer->AddActor(createLine(-_FIELD_WIDTH / 2, 0.0, 0.0, _FIELD_WIDTH / 2, 0.0, 0.0));
    _mainRenderer->AddActor(
            createLine(-_FIELD_WIDTH / 2, -_FIELD_LENGTH / 2, 0.0, -_FIELD_WIDTH / 2, _FIELD_LENGTH / 2, 0.0));
    _mainRenderer->AddActor(createLine(_FIELD_WIDTH / 2, -_FIELD_LENGTH / 2, 0.0, _FIELD_WIDTH / 2, _FIELD_LENGTH / 2, 0.0));
    _mainRenderer->AddActor(createLine(-_FIELD_WIDTH / 2, _FIELD_LENGTH / 2, 0.0, _FIELD_WIDTH / 2, _FIELD_LENGTH / 2, 0.0));
    _mainRenderer->AddActor(
            createLine(-_FIELD_WIDTH / 2, -_FIELD_LENGTH / 2, 0.0, _FIELD_WIDTH / 2, -_FIELD_LENGTH / 2, 0.0));

    // Goal Areas
    _mainRenderer->AddActor(
            createLine(-_GOAL_AREA_WIDTH / 2, -_FIELD_LENGTH / 2 + _GOAL_AREA_LENGTH, 0.0, _GOAL_AREA_WIDTH / 2,
                        -_FIELD_LENGTH / 2 + _GOAL_AREA_LENGTH, 0.0));
    _mainRenderer->AddActor(
            createLine(-_GOAL_AREA_WIDTH / 2, -_FIELD_LENGTH / 2, 0.0, -_GOAL_AREA_WIDTH / 2,
                        -_FIELD_LENGTH / 2 + _GOAL_AREA_LENGTH, 0.0));
    _mainRenderer->AddActor(
            createLine(_GOAL_AREA_WIDTH / 2, -_FIELD_LENGTH / 2, 0.0, _GOAL_AREA_WIDTH / 2,
                        -_FIELD_LENGTH / 2 + _GOAL_AREA_LENGTH, 0.0));
    _mainRenderer->AddActor(
            createLine(-_GOAL_AREA_WIDTH / 2, _FIELD_LENGTH / 2 - _GOAL_AREA_LENGTH, 0.0, _GOAL_AREA_WIDTH / 2,
                        _FIELD_LENGTH / 2 - _GOAL_AREA_LENGTH, 0.0));
    _mainRenderer->AddActor(
            createLine(-_GOAL_AREA_WIDTH / 2, _FIELD_LENGTH / 2, 0.0, -_GOAL_AREA_WIDTH / 2,
                        _FIELD_LENGTH / 2 - _GOAL_AREA_LENGTH, 0.0));
    _mainRenderer->AddActor(
            createLine(_GOAL_AREA_WIDTH / 2, _FIELD_LENGTH / 2, 0.0, _GOAL_AREA_WIDTH / 2,
                        _FIELD_LENGTH / 2 - _GOAL_AREA_LENGTH, 0.0));

    // Penalty Areas
    _mainRenderer->AddActor(
            createLine(-_PENALTY_AREA_WIDTH / 2, -_FIELD_LENGTH / 2 + _PENALTY_AREA_LENGTH, 0.0,
                        _PENALTY_AREA_WIDTH / 2, -_FIELD_LENGTH / 2 + _PENALTY_AREA_LENGTH, 0.0));
    _mainRenderer->AddActor(
            createLine(-_PENALTY_AREA_WIDTH / 2, -_FIELD_LENGTH / 2, 0.0, -_PENALTY_AREA_WIDTH / 2,
                        -_FIELD_LENGTH / 2 + _PENALTY_AREA_LENGTH, 0.0));
    _mainRenderer->AddActor(
            createLine(_PENALTY_AREA_WIDTH / 2, -_FIELD_LENGTH / 2, 0.0, _PENALTY_AREA_WIDTH / 2,
                        -_FIELD_LENGTH / 2 + _PENALTY_AREA_LENGTH, 0.0));
    _mainRenderer->AddActor(
            createLine(-_PENALTY_AREA_WIDTH / 2, _FIELD_LENGTH / 2 - _PENALTY_AREA_LENGTH, 0.0, _PENALTY_AREA_WIDTH / 2,
                        _FIELD_LENGTH / 2 - _PENALTY_AREA_LENGTH, 0.0));
    _mainRenderer->AddActor(
            createLine(-_PENALTY_AREA_WIDTH / 2, _FIELD_LENGTH / 2, 0.0, -_PENALTY_AREA_WIDTH / 2,
                        _FIELD_LENGTH / 2 - _PENALTY_AREA_LENGTH, 0.0));
    _mainRenderer->AddActor(
            createLine(_PENALTY_AREA_WIDTH / 2, _FIELD_LENGTH / 2, 0.0, _PENALTY_AREA_WIDTH / 2,
                        _FIELD_LENGTH / 2 - _PENALTY_AREA_LENGTH, 0.0));

    // Corner Circles
    renderArcAddActor(_mainRenderer, _FIELD_WIDTH / 2, _FIELD_LENGTH / 2, _CORNER_CIRCLE_RADIUS, 180, 270);
    renderArcAddActor(_mainRenderer, -_FIELD_WIDTH / 2, _FIELD_LENGTH / 2, _CORNER_CIRCLE_RADIUS, 270, 360);
    renderArcAddActor(_mainRenderer, -_FIELD_WIDTH / 2, -_FIELD_LENGTH / 2, _CORNER_CIRCLE_RADIUS, 0, 90);
    renderArcAddActor(_mainRenderer, _FIELD_WIDTH / 2, -_FIELD_LENGTH / 2, _CORNER_CIRCLE_RADIUS, 90, 180);

    // Center Circle
    renderArcAddActor(_mainRenderer, 0, 0, _CENTER_CIRCLE_RADIUS, 0, 360);

    // Black Dots
    renderDotAddActor(_mainRenderer, _FIELD_WIDTH / 4, 0, true);
    renderDotAddActor(_mainRenderer, -_FIELD_WIDTH / 4, 0, true);

    renderDotAddActor(_mainRenderer, _FIELD_WIDTH / 4, _FIELD_LENGTH / 2 - _PENALTY_MARK_DISTANCE, true);
    renderDotAddActor(_mainRenderer, -_FIELD_WIDTH / 4, _FIELD_LENGTH / 2 - _PENALTY_MARK_DISTANCE, true);
    renderDotAddActor(_mainRenderer, 0, _FIELD_LENGTH / 2 - _PENALTY_MARK_DISTANCE, false);

    renderDotAddActor(_mainRenderer, _FIELD_WIDTH / 4, -_FIELD_LENGTH / 2 + _PENALTY_MARK_DISTANCE, true);
    renderDotAddActor(_mainRenderer, -_FIELD_WIDTH / 4, -_FIELD_LENGTH / 2 + _PENALTY_MARK_DISTANCE, true);
    renderDotAddActor(_mainRenderer, 0, -_FIELD_LENGTH / 2 + _PENALTY_MARK_DISTANCE, false);

    renderDotAddActor(_mainRenderer, 0, 0, false, 0.1);

    _mainRenderer->AddActor(
            createLine(_PENALTY_AREA_WIDTH / 2, _FIELD_LENGTH / 2, 0.0, _PENALTY_AREA_WIDTH / 2,
                        _FIELD_LENGTH / 2 - _PENALTY_AREA_LENGTH, 0.0));
}

void FieldWidget3D::addGoalActors()
{
    // Get field dimensions
    double _FIELD_LENGTH = cEnvironmentField::getInstance().getLength();
    double post = cEnvironmentField::getInstance().getGoalPostWidth();

    vtkSmartPointer<vtkCubeSource> cubeSrc = vtkSmartPointer<vtkCubeSource>::New();
    cubeSrc->SetXLength(post);
    cubeSrc->SetYLength(post);
    cubeSrc->SetZLength(1);

    vtkSmartPointer<vtkCubeSource> cubeSrc2 = vtkSmartPointer<vtkCubeSource>::New();
    cubeSrc2->SetXLength(2 + 2 * post);
    cubeSrc2->SetYLength(post);
    cubeSrc2->SetZLength(post);

    vtkSmartPointer<vtkPolyDataMapper> goalMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    goalMapper->SetInputData(cubeSrc->GetOutput());

    vtkSmartPointer<vtkPolyDataMapper> goalMapper2 = vtkSmartPointer<vtkPolyDataMapper>::New();
    goalMapper2->SetInputData(cubeSrc2->GetOutput());

    vtkSmartPointer<vtkActor> goalBlue = vtkSmartPointer<vtkActor>::New();
    goalBlue->SetMapper(goalMapper);
    goalBlue->SetPosition(0 - 1 - post / 2, -_FIELD_LENGTH / 2 - post / 2, 0.5);
    goalBlue->GetProperty()->SetColor(0.2, 0.2, 1);
    goalBlue->GetProperty()->SetDiffuse(0.4);
    goalBlue->GetProperty()->SetAmbient(0.8);
    _mainRenderer->AddActor(goalBlue);

    vtkSmartPointer<vtkActor> goalBlue2 = vtkSmartPointer<vtkActor>::New();
    goalBlue2->SetMapper(goalMapper);
    goalBlue2->SetPosition(0 + 1 + post / 2, -_FIELD_LENGTH / 2 - post / 2, 0.5);
    goalBlue2->GetProperty()->SetColor(0.2, 0.2, 1);
    goalBlue2->GetProperty()->SetDiffuse(0.4);
    goalBlue2->GetProperty()->SetAmbient(0.8);
    _mainRenderer->AddActor(goalBlue2);

    vtkSmartPointer<vtkActor> goalBlue3 = vtkSmartPointer<vtkActor>::New();
    goalBlue3->SetMapper(goalMapper2);
    goalBlue3->SetPosition(0, -_FIELD_LENGTH / 2 - post / 2, 1);
    goalBlue3->GetProperty()->SetColor(0.2, 0.2, 1);
    goalBlue3->GetProperty()->SetDiffuse(0.4);
    goalBlue3->GetProperty()->SetAmbient(0.8);
    _mainRenderer->AddActor(goalBlue3);

    vtkSmartPointer<vtkActor> goalYellow = vtkSmartPointer<vtkActor>::New();
    goalYellow->SetMapper(goalMapper);
    goalYellow->SetPosition(0 - 1 - post / 2, _FIELD_LENGTH / 2 + post / 2, 0.5);
    goalYellow->GetProperty()->SetColor(1, 1, 0.2);
    goalYellow->GetProperty()->SetDiffuse(0.4);
    goalYellow->GetProperty()->SetAmbient(0.8);
    _mainRenderer->AddActor(goalYellow);

    vtkSmartPointer<vtkActor> goalYellow2 = vtkSmartPointer<vtkActor>::New();
    goalYellow2->SetMapper(goalMapper);
    goalYellow2->SetPosition(0 + 1 + post / 2, _FIELD_LENGTH / 2 + post / 2, 0.5);
    goalYellow2->GetProperty()->SetColor(1, 1, 0.2);
    goalYellow2->GetProperty()->SetDiffuse(0.4);
    goalYellow2->GetProperty()->SetAmbient(0.8);
    _mainRenderer->AddActor(goalYellow2);

    vtkSmartPointer<vtkActor> goalYellow3 = vtkSmartPointer<vtkActor>::New();
    goalYellow3->SetMapper(goalMapper2);
    goalYellow3->SetPosition(0, _FIELD_LENGTH / 2 + post / 2, 1);
    goalYellow3->GetProperty()->SetColor(1, 1, 0.2);
    goalYellow3->GetProperty()->SetDiffuse(0.4);
    goalYellow3->GetProperty()->SetAmbient(0.8);
    _mainRenderer->AddActor(goalYellow3);
}

void FieldWidget3D::addRobotActor(uint8_t robotId)
{
    _teamActors[robotId] = vtkSmartPointer<RobotVisualization>::New();
    _teamActors[robotId]->initialize(robotId, _mainRenderer);
    _mainRenderer->AddActor(_teamActors[robotId]);

    vtkSmartPointer<RobotLabel> label = vtkSmartPointer<RobotLabel>::New();
    label->initialize(robotId, _annotationRenderer, _teamActors[robotId]);
    _annotationRenderer->AddActor(label);
}

void FieldWidget3D::addObstacleActor()
{
    vtkSmartPointer<ObstacleVisualization> obst = vtkSmartPointer<ObstacleVisualization>::New();
    _obstacleActors.push_back(obst);
    _mainRenderer->AddActor(obst);
}

void FieldWidget3D::setObstaclePosition(PositionVelocity& posvel, ObjectId id)
{
    boost::format format_worldmodel("Enter FieldWidget3D::setObstaclePosition(%1%, %2%, %3%)");
    float x = posvel.x, y = posvel.y;
    int idAsInt = (int)id;
    TRACE(boost::str(format_worldmodel % x % y % idAsInt).c_str());

    // existing actor?
    int actorIndex = -1;
    if (_obstacleMapping.count(idAsInt))
    {
        actorIndex = _obstacleMapping[idAsInt].first;
    }
    else
    {
        // new tracker - assign first free (=invisible) actor
        for (size_t i = 0; i < _obstacleActors.size(); i++)
        {
            if (!_obstacleActors[i]->GetVisibility())
            {
                actorIndex = i;
                _obstacleMapping[idAsInt].first = actorIndex;
                break;
            }
        }
    }

    // update the actor
    if (actorIndex < 0)
    {
        // something went wrong! too few actors probably
        TRACE("failed to find an available obstacle actor for obstacle (id=%d, x=%6.2f, y=%6.2f)", idAsInt, x, y);
    }
    else
    {
        // update time stamp (keepalive)
        _obstacleMapping[idAsInt].second = _timestamp;
        // reposition actor
        _obstacleActors[actorIndex]->VisibilityOn();
        _obstacleActors[actorIndex]->setPosition(posvel);
    }
}

void FieldWidget3D::addTechnicalTeamAreaActor()
{
    _technicalTeamAreaActor = vtkSmartPointer<TechnicalTeamAreaVisualization>::New();
    _mainRenderer->AddActor(_technicalTeamAreaActor);
}

void FieldWidget3D::updateTechnicalTeamArea()
{
    if (_technicalTeamAreaActor != NULL)
    {
        _technicalTeamAreaActor->update();
    }
}

void FieldWidget3D::addForbiddenAreaActor()
{
    vtkSmartPointer<ForbiddenAreaVisualization> forbiddenArea = vtkSmartPointer<ForbiddenAreaVisualization>::New();
    _forbiddenAreaActors.push_back(forbiddenArea);
    _mainRenderer->AddActor(forbiddenArea);
}

void FieldWidget3D::setForbiddenAreaPosition(polygon2D& area, ObjectId id)
{
    int idAsInt = (int)id;

    // existing actor?
    int actorIndex = -1;
    if (_forbiddenAreaMapping.count(idAsInt))
    {
        actorIndex = _forbiddenAreaMapping[idAsInt].first;
    }
    else
    {
        // new tracker - assign first free (=invisible) actor
        for (size_t i = 0; i < _forbiddenAreaActors.size(); i++)
        {
            if (!_forbiddenAreaActors[i]->GetVisibility())
            {
                actorIndex = i;
                _forbiddenAreaMapping[idAsInt].first = actorIndex;
                break;
            }
        }
    }

    // update the actor
    if (actorIndex < 0)
    {
        // something went wrong! too few actors probably
        TRACE("failed to find an available actor for forbidden area (id=%d)",
                idAsInt);
    }
    else
    {
        // update time stamp (keepalive)
        _forbiddenAreaMapping[idAsInt].second = _timestamp;
        // reposition actor
        _forbiddenAreaActors[actorIndex]->VisibilityOn();
        _forbiddenAreaActors[actorIndex]->setPosition(area);
    }
}

void FieldWidget3D::addShootTargetActor(uint8_t robotId)
{
    _shootTargetActors[robotId] = vtkSmartPointer<ShootTargetVisualization>::New();
    _mainRenderer->AddActor(_shootTargetActors[robotId]);
}

void FieldWidget3D::setShootTargetPosition(uint8_t robotId, PositionVelocity& posvel)
{
    _shootTargetActors[robotId]->setPosition(posvel);
}

void FieldWidget3D::hideShootTarget(uint8_t robotId)
{
    _shootTargetActors[robotId]->visibilityOff();
}

void FieldWidget3D::addBallActor()
{
    vtkSmartPointer<BallVisualization> ball = vtkSmartPointer<BallVisualization>::New();
    _ballActors.push_back(ball);
    _mainRenderer->AddActor(ball);
}

void FieldWidget3D::addProjectSpeedActor()
{
    vtkSmartPointer<projectSpeedVisualization> speedVector = vtkSmartPointer<projectSpeedVisualization>::New();
    _projectSpeedActors.push_back(speedVector);
    _mainRenderer->AddActor(speedVector);
}

void FieldWidget3D::setProjectSpeedPosition(linepoint2D& speedVector, ObjectId id)
{
    int idAsInt = (int)id;

    // existing actor?
    int actorIndex = -1;
    if (_projectSpeedMapping.count(idAsInt))
    {
        actorIndex = _projectSpeedMapping[idAsInt].first;
    }
    else
    {
        // new tracker - assign first free (=invisible) actor
        for (size_t i = 0; i < _projectSpeedActors.size(); i++)
        {
            if (!_projectSpeedActors[i]->GetVisibility())
            {
                actorIndex = i;
                _projectSpeedMapping[idAsInt].first = actorIndex;
                break;
            }
        }
    }

    // update the actor
    if (actorIndex < 0)
    {
        // something went wrong! too few actors probably
        TRACE("failed to find an available actor for project speed vector (id=%d)",
                idAsInt);
    }
    else
    {
        // update time stamp (keepalive)
        _projectSpeedMapping[idAsInt].second = _timestamp;
        // reposition actor
        _projectSpeedActors[actorIndex]->VisibilityOn();
        _projectSpeedActors[actorIndex]->setPosition(speedVector);
    }
}

void FieldWidget3D::setBallPosition(PositionVelocity& posvel, ObjectId id, float confidence, float age, CameraType camType)
{
    boost::format format_worldmodel("Enter FieldWidget3D::setBallPosition(%1%, %2%, %3%)");
    float x = posvel.x, y = posvel.y;
    int idAsInt = (int)id;
    TRACE(boost::str(format_worldmodel % x % y % idAsInt).c_str());

    // existing actor?
    int actorIndex = -1;
    if (_ballMapping.count(idAsInt))
    {
        actorIndex = _ballMapping[idAsInt].first;
    }
    else
    {
        // new tracker - assign first free (=invisible) actor
        for (size_t i = 0; i < _ballActors.size(); i++)
        {
            if (!_ballActors[i]->GetVisibility())
            {
                actorIndex = i;
                _ballMapping[idAsInt].first = actorIndex;
                break;
            }
        }
    }

    // update the actor
    if (actorIndex < 0)
    {
        // something went wrong! too few actors probably
        TRACE("failed to find an available ball actor for ball (id=%d, x=%6.2f, y=%6.2f)", idAsInt, x, y);
    }
    else
    {
        // update time stamp (keepalive)
        _ballMapping[idAsInt].second = _timestamp;
        // reposition actor
        _ballActors[actorIndex]->VisibilityOn();
        _ballActors[actorIndex]->setPosition(posvel);
        TRACE("ball idx=%d cam=%d conf=%.2f age=%.2f", (int)actorIndex, (int)camType, confidence, age);
        // visualizer confidence and age using respectively color and opacity
        float redFactor = 0.0;
        if (confidence > 0.0) // for some measurements not even used
        {
            redFactor = 1.0 - confidence;
        }
        if (camType == FRONTVISION)
        {
            _ballActors[actorIndex]->setColor(CYAN, redFactor);
        }
        else
        {
            _ballActors[actorIndex]->setColor(YELLOW, redFactor);
        }
        _ballActors[actorIndex]->setOpacity(1.0 - age);
    }

    // TODO: why had old ball updater this unlock mutex, but obstacle not? _renderMutex.unlock();
}

void FieldWidget3D::addCollision(uint8_t robotId)
{
    // Create a sample collision
    vtkSmartPointer<CollisionBalloon> balloon = vtkSmartPointer<CollisionBalloon>::New();
    CollisionBalloonDimensions dim;
    dim.OuterRadiusX = 0.5;
    dim.OuterRadiusY = 0.3;
    dim.InnerRadiusX = dim.OuterRadiusX * 0.7;
    dim.InnerRadiusY = dim.OuterRadiusY * 0.6;
    balloon->initialize(_teamActors[robotId], dim);
    balloon->setColor(1, 0, 0);
    _mainRenderer->AddActor(balloon);

    // TODO remove the collision after x seconds
    // TODO: this somehow causes clipping of the z-plane? Thought I had solved this...
}

vtkSmartPointer<vtkActor> FieldWidget3D::createColoredDashedLine(float x1, float y1, float z1, float x2, float y2,
                                                                    float z2, double r, double g, double b)
{
    vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
    vtkSmartPointer<vtkPolyDataMapper> lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtkActor* lineActor = vtkActor::New();
    line->SetPoint1(x1, y1, z1);
    line->SetPoint2(x2, y2, z2);
    lineMapper->SetInputConnection(line->GetOutputPort());
    lineActor->SetMapper(lineMapper);
    lineActor->GetProperty()->SetLineWidth(3);
    lineActor->GetProperty()->SetLineStipplePattern(0xf0f0);
    lineActor->GetProperty()->SetLineStippleRepeatFactor(1);
    lineActor->GetProperty()->SetColor(r, g, b);
    lineActor->GetProperty()->SetPointSize(1);
    lineActor->GetProperty()->SetLineWidth(3);
    return lineActor;
}


vtkSmartPointer<vtkActor> FieldWidget3D::createColoredDot(float x, float y, float radius, double r, double g, double b)
{
    vtkSmartPointer<vtkCylinderSource> dot = vtkSmartPointer<vtkCylinderSource>::New();
    dot->SetRadius(radius);
    dot->SetHeight(0.001);
    dot->SetResolution(32);
    vtkSmartPointer<vtkPolyDataMapper> dotMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    dotMapper->SetInputData(dot->GetOutput());

    vtkActor* coloredDot = vtkActor::New();
    coloredDot->SetMapper(dotMapper);
    coloredDot->GetProperty()->SetColor(r, g, b);
    coloredDot->SetPosition(x, y, 0.01);
    coloredDot->SetOrientation(90, 0, 0);
    coloredDot->GetProperty()->SetAmbient(1.0);
    return coloredDot;
}

vtkActor* FieldWidget3D::createDashedLine(float x1, float y1, float z1, float x2, float y2, float z2)
{
    vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
    vtkSmartPointer<vtkPolyDataMapper> lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtkActor* lineActor = vtkActor::New();
    line->SetPoint1(x1, y1, z1);
    line->SetPoint2(x2, y2, z2);
    lineMapper->SetInputConnection(line->GetOutputPort());
    lineActor->SetMapper(lineMapper);
    lineActor->GetProperty()->SetLineWidth(3);
    lineActor->GetProperty()->SetLineStipplePattern(0xf0f0);
    lineActor->GetProperty()->SetLineStippleRepeatFactor(1);
    lineActor->GetProperty()->SetPointSize(1);
    lineActor->GetProperty()->SetLineWidth(3);
    return lineActor;
}

vtkSmartPointer<vtkActor> FieldWidget3D::createLine(float x1, float y1, float z1, float x2, float y2, float z2)
{
    vtkSmartPointer<vtkLineSource> line = vtkSmartPointer<vtkLineSource>::New();
    vtkSmartPointer<vtkPolyDataMapper> lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtkSmartPointer<vtkActor> lineActor = vtkSmartPointer<vtkActor>::New();
    line->SetPoint1(x1, y1, z1);
    line->SetPoint2(x2, y2, z2);
    lineMapper->SetInputConnection(line->GetOutputPort());
    lineActor->SetMapper(lineMapper);
    lineActor->GetProperty()->SetLineWidth(_LINE_THICKNESS);
    return lineActor;
}

void FieldWidget3D::renderArcAddActor(vtkRenderer *renderer, float x, float y, float radius, float startDeg, float endDeg)
{
    float x1, y1, x2, y2;
    x2 = x + radius * cos(startDeg * M_PI / 180);
    y2 = y + radius * sin(startDeg * M_PI / 180);
    for (int i = startDeg + 10; i <= endDeg; i += 10)
    {
        x1 = x + radius * cos(i * M_PI / 180);
        y1 = y + radius * sin(i * M_PI / 180);
        renderer->AddActor(createLine(x1, y1, 0, x2, y2, 0));
        x2 = x1;
        y2 = y1;
    }
}

void FieldWidget3D::renderDotAddActor(vtkRenderer* renderer, float x, float y, bool black, float radius)
{
    vtkSmartPointer<vtkCylinderSource> dot = vtkSmartPointer<vtkCylinderSource>::New();
    dot->SetRadius(radius);
    dot->SetHeight(0.001);
    dot->SetResolution(32);
    vtkSmartPointer<vtkPolyDataMapper> dotMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    dotMapper->SetInputData(dot->GetOutput());

    vtkSmartPointer<vtkActor> blackDot1 = vtkSmartPointer<vtkActor>::New();
    blackDot1->SetMapper(dotMapper);

    if (black)
    {
        blackDot1->GetProperty()->SetColor(0, 0, 0);
    }
    else
    {
        blackDot1->GetProperty()->SetColor(1, 1, 1);
    }

    blackDot1->SetPosition(x, y, 0.01);
    blackDot1->SetOrientation(90, 0, 0);
    blackDot1->GetProperty()->SetAmbient(1.0);
    renderer->AddActor(blackDot1);
}

void traceActor(vtkImageActor *actor, std::string name = "")
{
    double d[6];
    actor->GetPosition(d);
    TRACE("%s GetPosition %6.2f %6.2f %6.2f", name.c_str(), d[0], d[1], d[2]);
    actor->GetBounds(d);
    TRACE("%s GetBounds %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f", name.c_str(), d[0], d[1], d[2], d[3], d[4], d[5]);
    actor->GetOrientation(d);
    TRACE("%s GetOrientation %6.2f %6.2f %6.2f %6.2f", name.c_str(), d[0], d[1], d[2], d[3]);
    actor->GetScale(d);
    TRACE("%s GetScale %6.2f %6.2f %6.2f", name.c_str(), d[0], d[1], d[2]);
    actor->GetOrigin(d);
    TRACE("%s GetOrigin %6.2f %6.2f %6.2f", name.c_str(), d[0], d[1], d[2]);
}

void traceCam(vtkCamera *camera, std::string name = "")
{
    double d[4];
    camera->GetPosition(d);
    TRACE("%s GetPosition %6.2f %6.2f %6.2f", name.c_str(), d[0], d[1], d[2]);
    camera->GetFocalPoint(d);
    TRACE("%s GetFocalPoint %6.2f %6.2f %6.2f", name.c_str(), d[0], d[1], d[2]);
    camera->GetViewUp(d);
    TRACE("%s GetViewUp %6.2f %6.2f %6.2f", name.c_str(), d[0], d[1], d[2]);
    double *d2 = camera->GetOrientation();
    TRACE("%s GetOrientation %6.2f %6.2f %6.2f %6.2f", name.c_str(), d2[0], d2[1], d2[2], d2[3]);
}


void FieldWidget3D::updateWorldModelLocal(T_DIAG_WORLDMODEL_LOCAL& worldmodel_local, bool show_obstacle_measurements, bool show_balls_measurements)
{
    std::vector<diagObstacle>& obstacles = worldmodel_local.gaussianObstacleDiscriminatorData.obstacles;
    std::vector<diagMeasurement>& obs_meas = worldmodel_local.gaussianObstacleDiscriminatorData.measurements;
    std::vector<diagBall>& balls = worldmodel_local.gaussianBallDiscriminatorData.balls;
    std::vector<diagMeasurement3D>& ball_meas = worldmodel_local.gaussianBallDiscriminatorData.measurements;


    unsigned int obstacle_count = obstacles.size();
    unsigned int measurement_count = obs_meas.size();
    unsigned int ball_count = balls.size();
    unsigned int ball_measurement_count = ball_meas.size();
    unsigned int actor_count = _gaussianObstacleActors.size();

    // each ball has a position and velocity, so two gaussians are drawn
    int missing_actors = std::max(0u, (obstacle_count+measurement_count+(2*ball_count)+ball_measurement_count)-actor_count);

    for(int i=0; i<missing_actors; i++)
    {
        vtkSmartPointer<GaussianVisualization> obst_actor = vtkSmartPointer<GaussianVisualization>::New();
        _gaussianObstacleActors.push_back(obst_actor);
        _mainRenderer->AddActor(obst_actor);
    }

    std::vector<vtkSmartPointer<GaussianVisualization>>::iterator it_act = _gaussianObstacleActors.begin();
    std::vector<diagObstacle>::iterator it_obs = obstacles.begin();
    std::vector<diagMeasurement>::iterator it_meas = obs_meas.begin();
    std::vector<diagBall>::iterator it_ball = balls.begin();
    std::vector<diagMeasurement3D>::iterator it_ball_meas = ball_meas.begin();

    while(it_obs != obstacles.end())
    {
        (*it_act)->VisibilityOn();
        (*it_act)->setValue(it_obs->gaussian2d);
        (*it_act)->setColor(0.0, 0.0, 1.0);

        it_act++;
        it_obs++;
    }

    if(show_obstacle_measurements)
    {
        while(it_meas != obs_meas.end())
        {        
            (*it_act)->VisibilityOn();            
            (*it_act)->setValue(it_meas->gaussian2d);
            (*it_act)->setColor(1.0, 0.0, 0.0);

            it_act++;
            it_meas++;
        }
    }

    bool show_balls = true;
    if(show_balls)
    {
        while(it_ball != balls.end())
        {
            if(!it_ball->blacklisted || show_balls_measurements) // only show blacklisted balls if show_balls_measurements is active
            {
                (*it_act)->VisibilityOn();
                (*it_act)->setValue(it_ball->position);

                if(it_ball->bestBall)
                {
                    (*it_act)->setColor(1.0, 1.0, 0.5);
                }
                else if(!it_ball->blacklisted)
                {
                    (*it_act)->setColor(1.0, 1.0, 0.0);
                }
                else 
                {
                    (*it_act)->setColor(0.1, 0.1, 0.1);   
                }
                it_act++;
            }                

            // diagGaussian2D draw_velocity;
            // draw_velocity.mean = it_ball->position.mean;
            // draw_velocity.covariance = it_ball->velocity.covariance;
            // draw_velocity.covariance.matrix[0][0] *= 0.01;
            // draw_velocity.covariance.matrix[0][1] *= 0.01;
            // draw_velocity.covariance.matrix[1][0] *= 0.01;
            // draw_velocity.covariance.matrix[1][1] *= 0.01;

            // (*it_act)->VisibilityOn();
            // (*it_act)->setValue(draw_velocity);
            // (*it_act)->setColor(0.0, 1.0, 1.0);
            // it_act++;

            it_ball++;
        }
    }

    if(show_balls_measurements)
    {
        while(it_ball_meas != ball_meas.end())
        {
            (*it_act)->VisibilityOn();
            (*it_act)->setValue(it_ball_meas->gaussian3d);
            (*it_act)->setColor(0.8, 0.25, 0.0);
            it_act++;
            it_ball_meas++;
        }
    }

    while(it_act != _gaussianObstacleActors.end())
    {
        (*it_act)->VisibilityOff();
        it_act++;
    }

}

void FieldWidget3D::updateTrueBall(T_DIAG_TRUE_BALL& true_ball)
{
    _trueBallActor->VisibilityOn();
    PositionVelocity posvel;
    posvel.x = true_ball.x;
    posvel.y = true_ball.y;
    posvel.z = true_ball.z;
    _trueBallActor->setPosition(posvel);
    _trueBallActor->setColor(CYAN, 0.5);
    _trueBallActor->setOpacity(0.25);

    // setClockTick calls this same function, but is synchronized to the coach time
    // this leads to a delay on MATCH_STATE compared to the robots time
    // when analyzing the log of a specific robot, this signals is synchronized to
    // that robots time, leading to a more accurate image
    fieldVideoActor->updateImage(true_ball.timestamp.toDouble(), _heightmapName, _robotID);
}
