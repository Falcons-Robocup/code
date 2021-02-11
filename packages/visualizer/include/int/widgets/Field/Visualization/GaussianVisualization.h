// Copyright 2019-2020 lucas (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * GaussianVisualization.h
 *
 *  Created on: Sep 8, 2019
 *      Author: Lucas Catabriga
 */

#ifndef GAUSSIANVISUALIZATION_H
#define GAUSSIANVISUALIZATION_H

#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkAssembly.h>

// Internal:
#include "Visualization.h"

#include "diagWorldModelLocal.hpp"

/*
* Class that groups the actors belonging to a obstacle
*/
class GaussianVisualization : public Visualization
{
public:
    static GaussianVisualization* New()
    {
        return new GaussianVisualization();
    }

    GaussianVisualization();
    virtual ~GaussianVisualization() {};

    void setValue(const diagGaussian2D& gaussian_2d);
    void setValue(const diagGaussian3D& gaussian_3d);
    void setColor(double r, double g, double b);

private:
	vtkSmartPointer<vtkActor> fill_circle_actor[3];
	vtkSmartPointer<vtkActor> border_circle_actor[3];
    
};

#endif // GAUSSIANVISUALIZATION_H

