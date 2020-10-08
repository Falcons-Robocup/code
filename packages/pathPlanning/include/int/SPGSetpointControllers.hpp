 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * VelocitySetpointControllers.hpp
 *
 *  Created on: October, 2019
 *      Author: Jan Feitsma
 */

#ifndef PATHPLANNING_VELOCITYSETPOINTCONTROLLERS_HPP_
#define PATHPLANNING_VELOCITYSETPOINTCONTROLLERS_HPP_

// other packages
#include "falconsCommon.hpp" // Velocity2D, TODO #14
#include "tracing.hpp"

// own package
#include "AbstractVelocitySetpointController.hpp"
#include "facilities/PIDController.hpp" // TODO move to a common package?


// controller definitions, each implementation has its own file

class StopVelocitySetpointController : public AbstractVelocitySetpointController
{
public:
    bool calculate(PathPlanningVelocityControlData const &data, Velocity2D &resultVelocity);
};

// not useful for robot, but works OK for simulation
class LinearVelocitySetpointController : public AbstractVelocitySetpointController
{
public:
    bool calculate(PathPlanningVelocityControlData const &data, Velocity2D &resultVelocity);
};

// used on robot up to and including 2019
class PIDVelocitySetpointController : public AbstractVelocitySetpointController
{
public:
    bool calculate(PathPlanningVelocityControlData const &data, Velocity2D &resultVelocity);
private:
    // controllers need to remain state in between ticks, for integral term
    PIDController _controllerX;
    PIDController _controllerY;
    PIDController _controllerRz;
};

// SetPointGenerator, using Reflexxes motion control library
#include "SPGVelocitySetpointController.hpp"

struct AccelerationLimits
{
    float ax;
    float ay;
    float aRz;
};

class SPGVelocitySetpointController : public AbstractVelocitySetpointController
{
public:
    bool calculate(PathPlanningVelocityControlData const &data, Velocity2D &resultVelocity);
private:
    bool calculateUsingLimits(PathPlanningVelocityControlData const &data, Velocity2D &resultVelocity, AccelerationLimits const &accLimits);
};

#endif

