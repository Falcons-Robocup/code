// Copyright 2016-2019 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RobotVisualization.h
 *
 *  Created on: July 21, 2016
 *      Author: Jan Feitsma
 */

#ifndef ROBOTVISUALIZATION_H
#define ROBOTVISUALIZATION_H

#include <vtkRenderer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkSmartPointer.h>
#include <vtkFollower.h>
#include <vector>

#include <QTimer>

// Internal:
#include "Visualization.h"

/*
* Class that groups the actor belonging to a single friendly robot
*/
class RobotVisualization : public Visualization
{
        Q_OBJECT

public:
    static RobotVisualization* New()
    {
        return new RobotVisualization();
    }

    void initialize(int robotID, vtkRenderer *renderer);
    void setPosition(PositionVelocity& posvel);
    void setPath(std::vector<PositionVelocity>& path);
    void setPathPlanningEnabled(bool enabled); // Allows enabling/disabling of path processing
    void setStatus(int status);
    void hideGhosts();
    void setGhostPosition(int ghostId, PositionVelocity const&posvel);
    void showArrow();
    void hideArrow();

Q_SIGNALS:
    void signalPlannedPathChanged(std::vector<PositionVelocity>& path);

public Q_SLOTS:
    void blink();

private:
    int _robotID;
    bool _pathPlanningEnabled;
    vtkSmartPointer<vtkActor> _actor;
    std::vector<vtkSmartPointer<vtkActor>> _pathGhosts;
    QTimer* _blinkTimer = NULL;
    void blinkOn();
    void blinkOff();
};

#endif // ROBOTVISUALIZATION_H
