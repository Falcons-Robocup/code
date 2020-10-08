 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Visualization.cpp
 *
 *  Created on: November 22nd, 2016
 *      Author: Diana Koenraadt
 */

// Internal:
#include "int/widgets/Field/Visualization/Visualization.h"

// Falcons specific:
#include "vector2d.hpp"
#include "tracing.hpp"

Visualization::Visualization()
{
    // Create pointing arrow
    _arrowSource = vtkSmartPointer<vtkArrowSource>::New();
    _arrowSource->SetShaftRadius(0.05);
    _arrowSource->SetTipRadius(0.1);
    _arrowSource->SetTipLength(0.3);
    _arrow = addAsActor(_arrowSource);
}

PositionVelocity Visualization::getPosition()
{
    return _posvel;
}

void Visualization::setPosition(PositionVelocity& posvel)
{
    this->SetPosition(posvel.x, posvel.y, posvel.z);
    this->SetOrientation(0, 0, 180.0 * posvel.phi / M_PI); // radians to degrees
    _posvel = posvel;
    emit signalPositionChanged(posvel);
}

void Visualization::setPosition(polygon2D& area)
{
	this->SetPosition(0.0, 0.0, 0.0);
    _area = area;

    emit signalPositionChanged(area);
}

void Visualization::setPosition(linepoint2D& line)
{
	this->SetPosition(line.getSourcePoint2D().x, line.getSourcePoint2D().y, 0.1);
    _line = line;

    emit signalPositionChanged(line);
}

void Visualization::VisibilityOn()
{
    vtkAssembly::VisibilityOn();
    emit signalVisibilityChanged(true);
}

void Visualization::VisibilityOff()
{
    vtkAssembly::VisibilityOff();
    emit signalVisibilityChanged(false);
}
