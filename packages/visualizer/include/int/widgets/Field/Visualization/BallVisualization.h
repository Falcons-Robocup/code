// Copyright 2016-2017 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * BallVisualization.h
 *
 *  Created on: October 3rd, 2016
 *      Author: Diana Koenraadt
 */

#ifndef BALLVISUALIZATION_H
#define BALLVISUALIZATION_H

#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkAssembly.h>

// Internal:
#include "Visualization.h"

enum BallColor
{
    YELLOW,
    CYAN
};

/*
* Class that groups the actors belonging to a ball 
*/
class BallVisualization : public Visualization
{
public:
    static BallVisualization* New()
    {
        return new BallVisualization();
    }

    BallVisualization();
    virtual ~BallVisualization() {};

    double getDiameter();
    void setPosition(PositionVelocity& posvel);
    void setColor(BallColor color, float redFactor); // redFactor in [0,1] adds a bit of red for confidence visualization
    void setOpacity(float opacity);
};

#endif // BALLVISUALIZATION_H
