 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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

