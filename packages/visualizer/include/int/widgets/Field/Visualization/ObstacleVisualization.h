// Copyright 2017 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ObstacleVisualization.h
 *
 *  Created on: Mar 18, 2017
 *      Author: Jan Feitsma
 */

#ifndef OBSTACLEVISUALIZATION_H
#define OBSTACLEVISUALIZATION_H

#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkAssembly.h>

// Internal:
#include "Visualization.h"

/*
* Class that groups the actors belonging to a obstacle
*/
class ObstacleVisualization : public Visualization
{
public:
    static ObstacleVisualization* New()
    {
        return new ObstacleVisualization();
    }

    ObstacleVisualization();
    virtual ~ObstacleVisualization() {};

    double getDiameter();
    void setPosition(PositionVelocity& posvel);
    
};

#endif // OBSTACLEVISUALIZATION_H

