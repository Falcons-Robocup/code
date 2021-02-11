// Copyright 2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ForbiddenAreaVisualization.h
 *
 *  Created on: Apr 25, 2017
 *      Author: Tim Kouters
 */

#ifndef FORBIDDENAREAVISUALIZATION_H
#define FORBIDDENAREAVISUALIZATION_H

#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkAssembly.h>

// Internal:
#include "Visualization.h"

// External:
#include "polygon2D.hpp"

/*
* Class that groups the actors belonging to a forbidden area
*/
class ForbiddenAreaVisualization : public Visualization
{
public:
    static ForbiddenAreaVisualization* New()
    {
        return new ForbiddenAreaVisualization();
    }

    ForbiddenAreaVisualization();
    virtual ~ForbiddenAreaVisualization() {};

    void setPosition(polygon2D& area);
    
};

#endif // FORBIDDENAREAVISUALIZATION_H

