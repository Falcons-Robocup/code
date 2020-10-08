 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
