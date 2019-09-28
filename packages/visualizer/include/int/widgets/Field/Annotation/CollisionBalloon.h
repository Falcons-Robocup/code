 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
