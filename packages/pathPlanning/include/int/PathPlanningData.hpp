 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * PathPlanningData.hpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#ifndef PATHPLANNINGDATA_HPP_
#define PATHPLANNINGDATA_HPP_

// system
#include <vector>

// other Falcons packages
#include "falconsCommon.hpp" // TODO disentangle (move Position2D etc. to geometry package? #14)

// sharedTypes
#include "actionResult.hpp"
#include "ConfigPathPlanning.hpp"
#include "diagPathPlanning.hpp"
#include "wayPoint.hpp"
#include "forbiddenArea.hpp"
#include "motionSetpoint.hpp"
#include "obstacleResult.hpp"
#include "ballResult.hpp"
#include "robotState.hpp"
#include "robotVelocity.hpp"


struct PathPlanningData
{
    // inputs
    ConfigPathPlanning          config;
    wayPoint                    target;
    rtime                       timestamp;
    std::vector<forbiddenArea>  forbiddenAreas;
    robotState                  robot;
    std::vector<ballResult>     balls;
    std::vector<robotState>     teamMembers;
    std::vector<obstacleResult> obstacles; // only the ones from worldModel, see calculatedObstacles below

    // calculation results
    std::vector<wayPoint>       path;
    actionResultTypeEnum        resultStatus;
    VelocitySetpointControllerConfig vcConfig;
    std::vector<forbiddenArea>  calculatedForbiddenAreas; // = input + obstacle paths
    std::vector<obstacleResult> calculatedObstacles;
    Position2D                  targetPositionFcs; // might be corrected with ball possession offset
    Position2D                  currentPositionFcs; // might be corrected with ball possession offset
    Velocity2D                  currentVelocityFcs; // might be corrected with ball possession offset
    Position2D                  deltaPositionFcs; // delta of current position w.r.t. subtarget (=first waypoint)
    Position2D                  deltaPositionRcs;
    bool                        shortStroke;
    Velocity2D                  resultVelocityRcs;
    pose                        accelerationRcs; // TODO #14 make consistent
    bool                        isAccelerating[3];
    bool                        accelerationClipping[3];
    bool                        deadzone[3];

    // internal data
    float                       dt;
    bool                        done = false;
    diagPIDstate                pidState; // PID-specific analysis
    bool                        slow = false;
    bool                        stop = false;
    MotionLimitsConfigPP        currentLimits; // switching on setpiece/slow flag and on ball possession
    Velocity2D                  previousVelocityRcs; // store from previous iteration, for acceleration limiter
    rtime                       previousTimestamp;

    // temporary Rz limiter override
    bool                        overrideRzLimits = false;
    float                       overrideRzMaxVel = 0.0;
    float                       overrideRzMaxAcc = 0.0;

    // functions
    void reset();
    void traceInputs();
    void traceOutputs();
    Position2D getSubTarget() const;
    void insertSubTarget(Position2D const &pos, Velocity2D const &vel = Velocity2D(0,0,0));
    void addForbiddenAreas(std::vector<forbiddenArea> const &newForbiddenAreas);
    void addForbiddenArea(forbiddenArea const &newForbiddenArea);
    void configureLimits();
};

// substructure for the VelocityController interface
struct PathPlanningVelocityControlData
{
    Position2D                  deltaPosition; // in RCS or FCS depending on configuration
    Velocity2D                  currentVelocity; // in RCS or FCS depending on configuration
    Velocity2D                  previousVelocitySetpoint;
    float                       dt;
    MotionLimitsConfigPP        currentLimits;
    diagPIDstate                pidState; // PID-specific analysis
};

#endif

