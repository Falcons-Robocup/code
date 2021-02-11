// Copyright 2016-2017 Diana Koenraadt (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RobotLabel.h
 *
 *  Created on: November 26, 2016
 *      Author: Diana Koenraadt
 *
 * "In case our robots bump into eachother or into the boundary, this should be visualized. Also, when an opponent runs into us, 
 *  this should be visualized so that we can complain to the referee."
 */

#ifndef COLLISIONBALLOON_H
#define COLLISIONBALLOON_H

#include <vtkAssembly.h>
#include <sstream>

// Internal:
#include "int/widgets/Field/Visualization/RobotVisualization.h"
#include "Annotation.h"

struct CollisionBalloonDimensions
{
    double InnerRadiusX = 0.1;
    double InnerRadiusY = 0.1;
    double OuterRadiusX = 0.2;
    double OuterRadiusY = 0.2;
};

/*
* Class that draws a balloon at the spot where it was first created
*/
class CollisionBalloon : public Annotation, public vtkAssembly
{
public:
    static CollisionBalloon* New()
    {
        return new CollisionBalloon();
    }

    void initialize(RobotVisualization* anchor, CollisionBalloonDimensions dim);
    void setColor(double red, double green, double blue); // Set the color. Default color is red.

public Q_SLOTS:
    virtual void onAnchorVisibilityChanged(bool visible) override;

private:
    void addBalloon(CollisionBalloonDimensions dim);
    bool _posvelSet = false; // True after the position has been set once; position of collision balloon does not change after that.
    vtkSmartPointer<vtkActor> _actor;
};

#endif // COLLISIONBALLOON_H
