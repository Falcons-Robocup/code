// Copyright 2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * projectSpeedVisualization.h
 *
 *  Created on: Jul 28, 2017
 *      Author: Tim Kouters
 */

#ifndef PROJECTSPEEDVISUALIZATION_H
#define PROJECTSPEEDVISUALIZATION_H

#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkAssembly.h>

// Internal:
#include "Visualization.h"

// External:
#include "linepoint2D.hpp"

/*
* Class that groups the actors belonging to a forbidden area
*/
class projectSpeedVisualization : public Visualization
{
	public:
		static projectSpeedVisualization* New()
		{
			return new projectSpeedVisualization();
		}

		projectSpeedVisualization();
		virtual ~projectSpeedVisualization() {};

		void setPosition(linepoint2D& line);
    
};

#endif // PROJECTSPEEDVISUALIZATION_H

