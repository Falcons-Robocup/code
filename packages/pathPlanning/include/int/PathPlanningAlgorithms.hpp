 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * PathPlanningAlgorithms.hpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#ifndef PATHPLANNING_ALGORITHMS_HPP_
#define PATHPLANNING_ALGORITHMS_HPP_


#include "int/PathPlanningData.hpp"
#include "int/AbstractVelocitySetpointController.hpp"
#include "tracing.hpp"

/*!
 * \brief is the abstract class that each PathPlanning algorithm should inherit.
 *
 */
class PathPlanningAlgorithm
{
public:
    PathPlanningAlgorithm() {}

    virtual void execute(PathPlanningData &data) = 0;
};


// algorithm definitions, each implementation has its own file

class RequireWorldModelActive : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class CheckStopCommand : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class CheckTargetValid : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class CheckTargetReached : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class Shielding : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class ForwardDriving : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class CalculateDeltas : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class CalculateObstacles : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class CalculateAccelerating : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class SelectVelocityController : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class CalculateVelocity : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);

    // callback to get VelocitySetpointController
    // link from algorithm to PathPlanning class
    boost::function<AbstractVelocitySetpointController *(void)> _callback;

public:
    CalculateVelocity(boost::function<AbstractVelocitySetpointController *(void)> callback);
};

class Deadzone : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class ApplyLimits : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class EscapeForbiddenAreas : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class AvoidObstacles : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class ShiftBallOffset : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

class UnShiftBallOffset : public PathPlanningAlgorithm
{
    void execute(PathPlanningData &data);
};

#endif
