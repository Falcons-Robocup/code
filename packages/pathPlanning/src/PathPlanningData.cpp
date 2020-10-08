 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * PathPlanningData.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#include "int/PathPlanningData.hpp"
#include "tracing.hpp"
#include "generated_enum2str.hpp"


void PathPlanningData::reset()
{
    TRACE_FUNCTION("");
    // clear calculation results and internal data; inputs are overridden at start of iteration
    // make sure all diagnostics related items are cleared here, otherwise you might end up diagnosing data from an older iteration
    path.clear();
    calculatedObstacles.clear();
    calculatedForbiddenAreas.clear();
    resultStatus = actionResultTypeEnum::INVALID;
    deltaPositionFcs = Position2D(0.0, 0.0, 0.0);
    deltaPositionRcs = Position2D(0.0, 0.0, 0.0);
    resultVelocityRcs = Velocity2D(0.0, 0.0, 0.0);
    accelerationRcs = pose();
    pidState = diagPIDstate();
    for (int dof = 0; dof < 3; ++dof)
    {
        isAccelerating[dof] = false;
        accelerationClipping[dof] = false;
        deadzone[dof] = false;
    }
    done = false;
    slow = false;
    stop = false;
}

void PathPlanningData::traceInputs()
{
    TRACE_FUNCTION("");
    TRACE("robotPos=[%6.2f, %6.2f, %6.2f] robotVel=[%6.2f, %6.2f, %6.2f]", robot.position.x, robot.position.y, robot.position.Rz, robot.velocity.x, robot.velocity.y, robot.velocity.Rz);
    TRACE("stop=%d targetPos=[%6.2f, %6.2f, %6.2f] slow=%d", stop, target.pos.x, target.pos.y, target.pos.Rz, slow);
}

void PathPlanningData::traceOutputs()
{
    TRACE_FUNCTION("");
    if (path.size())
    {
        auto subtarget = path.at(0);
        TRACE("result=%-8s  ROBOT_VELOCITY_SETPOINT=[%6.2f, %6.2f, %6.2f] subTargetPos=[%6.2f, %6.2f, %6.2f]", enum2str(resultStatus), resultVelocityRcs.x, resultVelocityRcs.y, resultVelocityRcs.phi, subtarget.pos.x, subtarget.pos.y, subtarget.pos.Rz);
    }
    else
    {
        TRACE("result=%-8s  ROBOT_VELOCITY_SETPOINT=[%6.2f, %6.2f, %6.2f] subTargetPos=NONE", enum2str(resultStatus), resultVelocityRcs.x, resultVelocityRcs.y, resultVelocityRcs.phi);
    }
}

void PathPlanningData::insertSubTarget(Position2D const &pos, Velocity2D const &vel)
{
    TRACE_FUNCTION("");
    // only insert if subtarget is sufficiently far away
    Position2D deltaPositionFcsLocal = pos - currentPositionFcs;
    deltaPositionFcsLocal.phi = project_angle_mpi_pi(deltaPositionFcsLocal.phi);
    bool xyFar = deltaPositionFcsLocal.xy().size() >= currentLimits.toleranceXY;
    bool RzFar = fabs(deltaPositionFcsLocal.phi) >= currentLimits.toleranceRz;
    TRACE("xyFar=%d RzFar=%d", xyFar, RzFar);
    if (xyFar || RzFar)
    {
        wayPoint wp;
        wp.pos.x = pos.x;
        wp.pos.y = pos.y;
        wp.pos.Rz = pos.phi;
        wp.vel.x = vel.x;
        wp.vel.y = vel.y;
        wp.vel.Rz = vel.phi;
        path.insert(path.begin(), wp);
        Position2D p = pos;
        Velocity2D v = vel;
        TRACE("adding subtarget %s %s, path size is now %d", p.tostr(), v.tostr(), (int)path.size());
    }
    else
    {
        TRACE("ignoring new subtarget because it is too close by");
    }
}

void PathPlanningData::addForbiddenAreas(std::vector<forbiddenArea> const &newForbiddenAreas)
{
    TRACE_FUNCTION("");
    TRACE("n=%d", (int)newForbiddenAreas.size());
    // give each one a unique id, for diagnostics, and to prevent visualizer confusion
    for (int it = 0; it < (int)newForbiddenAreas.size(); ++it)
    {
        addForbiddenArea(newForbiddenAreas.at(it));
    }
}

void PathPlanningData::addForbiddenArea(forbiddenArea const &newForbiddenArea)
{
    TRACE_FUNCTION("");
    // give each one a unique id, for diagnostics, and to prevent visualizer confusion
    auto f = newForbiddenArea;
    f.id = (int)calculatedForbiddenAreas.size();
    TRACE("id=%d %6.2f", f.id, f.points[0].x);
    calculatedForbiddenAreas.push_back(f);
}

Position2D PathPlanningData::getSubTarget() const
{
    Position2D result;
    if (path.size())
    {
        auto subtarget = path.at(0);
        result.x = subtarget.pos.x;
        result.y = subtarget.pos.y;
        result.phi = subtarget.pos.Rz;
    }
    else
    {
        result.x = robot.position.x;
        result.y = robot.position.y;
        result.phi = robot.position.Rz;
    }
    return result;
}

void PathPlanningData::configureLimits()
{
    currentLimits.enabled = config.limits.enabled;
    currentLimits.maxDecX = config.limits.common.maxDecX;
    currentLimits.maxDecY = config.limits.common.maxDecY;
    currentLimits.maxDecRz = config.limits.common.maxDecRz;
    currentLimits.toleranceXY = config.limits.common.toleranceXY;
    currentLimits.toleranceRz = config.limits.common.toleranceRz;
    currentLimits.accThresholdX = config.limits.common.accThresholdX;
    currentLimits.accThresholdY = config.limits.common.accThresholdY;
    currentLimits.accThresholdRz = config.limits.common.accThresholdRz;
    auto specificLimits = (robot.hasBall ? config.limits.withBall : config.limits.withoutBall);
    currentLimits.maxVelX = specificLimits.maxVelX;
    currentLimits.maxVelYforward = specificLimits.maxVelYforward;
    currentLimits.maxVelYbackward = specificLimits.maxVelYbackward;
    currentLimits.maxVelRz = specificLimits.maxVelRz;
    currentLimits.maxAccX = specificLimits.maxAccX;
    currentLimits.maxAccYforward = specificLimits.maxAccYforward;
    currentLimits.maxAccYbackward = specificLimits.maxAccYbackward;
    currentLimits.maxAccRz = specificLimits.maxAccRz;
    if (slow)
    {
        float f = config.slowFactor;
        currentLimits.maxVelX *= f;
        currentLimits.maxVelYforward *= f;
        currentLimits.maxVelYbackward *= f;
        currentLimits.maxVelRz *= f;
        currentLimits.maxAccX *= f;
        currentLimits.maxAccYforward *= f;
        currentLimits.maxAccYbackward *= f;
        currentLimits.maxAccRz *= f;
    }
    TRACE("limits have been configured to: %s", tostr(currentLimits).c_str());
}

