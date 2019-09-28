 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPathPlanningData.cpp
 *
 *  Created on: Nov 17, 2015
 *      Author: Erik Kouters
 */

#include <boost/thread/thread.hpp>
#include "int/cPathPlanningData.hpp"
#include "int/adapters/cDiagnosticsAdapter.hpp"
#include "cEnvironmentField.hpp"
#include "cDiagnostics.hpp"
#include "int/facilities/cObstacleFacilities.hpp"

boost::mutex _mtx;

cPathPlanningData::cPathPlanningData()
{
    _logged_out_of_bounds = false;
}

// Main data
void cPathPlanningData::getAlgorithmType(pp_algorithm_type& algo)
{
    _mtx.lock();
    algo = _algType;
    _mtx.unlock();
}
void cPathPlanningData::setAlgorithmType(const pp_algorithm_type& algo)
{

    _mtx.lock();
    _algType = algo;
    _mtx.unlock();

}

void cPathPlanningData::getMotionProfileType(pp_motionProfile_type& motionProfile)
{
    _mtx.lock();
    motionProfile = _motionProfile;
    _mtx.unlock();
}
void cPathPlanningData::setMotionProfileType(const pp_motionProfile_type& motionProfile)
{

    _mtx.lock();
    _motionProfile = motionProfile;
    _mtx.unlock();

}

void cPathPlanningData::getRobotStop(bool& stop)
{
    _mtx.lock();
    stop = _robotStop;
    _mtx.unlock();
}
void cPathPlanningData::setRobotStop(const bool& stop)
{
    _mtx.lock();
    _robotStop = stop;
    _mtx.unlock();
}




// WorldModel data
void cPathPlanningData::getPosition(Position2D& pos)
{

    _mtx.lock();
    pos = _pos;
    _mtx.unlock();
}
void cPathPlanningData::setPosition(const Position2D& pos)
{

    _mtx.lock();
    _pos = pos;
    _mtx.unlock();
}

void cPathPlanningData::getVelocity(Velocity2D& vel)
{

    _mtx.lock();
    vel = _vel;
    _mtx.unlock();
}
void cPathPlanningData::setVelocity(const Velocity2D& vel)
{

    _mtx.lock();
    _vel = vel;
    _mtx.unlock();
}

void cPathPlanningData::getAcceleration(Velocity2D& acc)
{

    _mtx.lock();
    acc = _acc;
    _mtx.unlock();
}
void cPathPlanningData::setAcceleration(const Velocity2D& acc)
{

    _mtx.lock();
    _acc = acc;
    _mtx.unlock();
}

void cPathPlanningData::getOnlyObstacles(std::vector<pp_obstacle_struct_t>& obstacles)
{

    _mtx.lock();
    obstacles = _obstacles;
    _mtx.unlock();
}

void cPathPlanningData::getAllObstacles(std::vector<pp_obstacle_struct_t>& obstacles)
{

    _mtx.lock();
    obstacles = _obstacles;

    /* Adding forbidden areas as obstacles as well */
    for(size_t i = 0; i < _staticForbiddenAreas.size(); i++)
    {
        std::vector<pp_obstacle_struct_t> forbiddenObstacles;
        cObstacleFacilities::projectObstaclesOnForbiddenArea(_staticForbiddenAreas.at(i), obstacles);
        obstacles.insert(obstacles.end(), forbiddenObstacles.begin(), forbiddenObstacles.end());
    }

    for(size_t i = 0; i < _dynamicForbiddenAreas.size(); i++)
    {
        std::vector<pp_obstacle_struct_t> forbiddenObstacles;
        cObstacleFacilities::projectObstaclesOnForbiddenArea(_dynamicForbiddenAreas.at(i), obstacles);
        obstacles.insert(obstacles.end(), forbiddenObstacles.begin(), forbiddenObstacles.end());
    }

    _mtx.unlock();
}

void cPathPlanningData::setObstacles(const std::vector<pp_obstacle_struct_t>& obstacles)
{
    _mtx.lock();
    _obstacles = obstacles;
    _mtx.unlock();

}

void cPathPlanningData::getForbiddenAreas(std::vector<pp_obstacle_struct_t>& forbiddenAreas)
{

    _mtx.lock();

    forbiddenAreas.clear();

    for(size_t i = 0; i < _staticForbiddenAreas.size(); i++)
    {
        std::vector<pp_obstacle_struct_t> obstacles;
        cObstacleFacilities::projectObstaclesOnForbiddenArea(_staticForbiddenAreas.at(i), obstacles);
        forbiddenAreas.insert(forbiddenAreas.end(), obstacles.begin(), obstacles.end());
    }

    for(size_t i = 0; i < _dynamicForbiddenAreas.size(); i++)
    {
        std::vector<pp_obstacle_struct_t> obstacles;
        cObstacleFacilities::projectObstaclesOnForbiddenArea(_dynamicForbiddenAreas.at(i), obstacles);
        forbiddenAreas.insert(forbiddenAreas.end(), obstacles.begin(), obstacles.end());
    }

    _mtx.unlock();
}

void cPathPlanningData::getForbiddenAreas(std::vector<polygon2D>& forbiddenAreas)
{

    _mtx.lock();
    forbiddenAreas.clear();

    forbiddenAreas.insert(forbiddenAreas.end(), _staticForbiddenAreas.begin(), _staticForbiddenAreas.end());
    forbiddenAreas.insert(forbiddenAreas.end(), _dynamicForbiddenAreas.begin(), _dynamicForbiddenAreas.end());

    _mtx.unlock();
}

void cPathPlanningData::setStaticForbiddenAreas(const std::vector<polygon2D>& forbiddenAreas)
{
    _mtx.lock();
    _staticForbiddenAreas = forbiddenAreas;
    _mtx.unlock();

}

void cPathPlanningData::setDynamicForbiddenAreas(const std::vector<polygon2D>& forbiddenAreas)
{
    _mtx.lock();
    _dynamicForbiddenAreas = forbiddenAreas;
    _mtx.unlock();

}

void cPathPlanningData::getRobotActive(bool& active)
{

    _mtx.lock();
    active = _robotActive;
    _mtx.unlock();
}
void cPathPlanningData::setRobotActive(const bool& active)
{
    _mtx.lock();
    _robotActive = active;
    _mtx.unlock();

}

void cPathPlanningData::getHaveBall(bool& haveBall)
{

    _mtx.lock();
    haveBall = _haveBall;
    _mtx.unlock();

}
void cPathPlanningData::setHaveBall(const bool& haveBall)
{

    _mtx.lock();
    _haveBall = haveBall;
    _mtx.unlock();

}

void cPathPlanningData::getBallPos(Position2D& ballPos)
{

    _mtx.lock();
    ballPos = _ballPos;
    _mtx.unlock();

}
void cPathPlanningData::setBallPos(const Position2D& ballPos)
{

    _mtx.lock();
    _ballPos = ballPos;
    _mtx.unlock();

}

void cPathPlanningData::getBallValid(bool& ballValid)
{

    _mtx.lock();
    ballValid = _ballValid;
    _mtx.unlock();

}
void cPathPlanningData::setBallValid(const bool& ballValid)
{

    _mtx.lock();
    _ballValid = ballValid;
    _mtx.unlock();

}

void cPathPlanningData::getDeltaPosition(Position2D& pos)
{

    _mtx.lock();
    pos = _deltaPos;
    _mtx.unlock();
}
void cPathPlanningData::setDeltaPosition(const Position2D& pos)
{

    _mtx.lock();
    _deltaPos = pos;
    _mtx.unlock();
}

void cPathPlanningData::getIsLowestActiveRobot(bool &isLowest)
{
    _mtx.lock();
    isLowest = _robotIsLowestActive;
    _mtx.unlock();
}

void cPathPlanningData::setIsLowestActiveRobot(const bool &isLowest)
{
    _mtx.lock();
    _robotIsLowestActive = isLowest;
    _mtx.unlock();
}


// Reconfigure data

void cPathPlanningData::setConfig(configPathPlanning const &config)
{
    // call relevant smaller setters below, rewriting data type
    setObstacleAvoidanceIsEnabled(config.obstacleAvoidanceEnabled);

    pp_limiters_struct_t limits;
    limits.maxVelXY                            = config.normal.limiters.maxVelXY;
    limits.maxVelXY_withBall                   = config.normal.limiters.maxVelXY_withBall;
    limits.maxVelPhi                           = config.normal.limiters.maxVelPhi;
    limits.maxVelPhi_withBall                  = config.normal.limiters.maxVelPhi_withBall;
    limits.maxAccXY                            = config.normal.limiters.maxAccXY;
    limits.maxAccPhi                           = config.normal.limiters.maxAccPhi;
    limits.tolerationXY                        = config.normal.limiters.tolerationXY;
    limits.tolerationPhi                       = config.normal.limiters.tolerationPhi;
    limits.relativeSpeedFactorX                = config.normal.limiters.relativeSpeedFactorX;
    limits.relativeSpeedFactorY                = config.normal.limiters.relativeSpeedFactorY;
    limits.relativeSpeedFactorPhi              = config.normal.limiters.relativeSpeedFactorPhi;
    limits.obstacleAvoidanceScalingFactor      = config.normal.limiters.obstacleAvoidanceScalingFactor;
    limits.obstacleAvoidanceDistanceFactor     = config.normal.limiters.obstacleAvoidanceDistanceFactor;
    setLimits(pp_motionProfile_type::NORMAL, limits);

    limits.maxVelXY                            = config.setpiece.limiters.maxVelXY;
    limits.maxVelXY_withBall                   = config.setpiece.limiters.maxVelXY_withBall;
    limits.maxVelPhi                           = config.setpiece.limiters.maxVelPhi;
    limits.maxVelPhi_withBall                  = config.setpiece.limiters.maxVelPhi_withBall;
    limits.maxAccXY                            = config.setpiece.limiters.maxAccXY;
    limits.maxAccPhi                           = config.setpiece.limiters.maxAccPhi;
    limits.tolerationXY                        = config.setpiece.limiters.tolerationXY;
    limits.tolerationPhi                       = config.setpiece.limiters.tolerationPhi;
    limits.relativeSpeedFactorX                = config.setpiece.limiters.relativeSpeedFactorX;
    limits.relativeSpeedFactorY                = config.setpiece.limiters.relativeSpeedFactorY;
    limits.relativeSpeedFactorPhi              = config.setpiece.limiters.relativeSpeedFactorPhi;
    limits.obstacleAvoidanceScalingFactor      = config.setpiece.limiters.obstacleAvoidanceScalingFactor;
    limits.obstacleAvoidanceDistanceFactor     = config.setpiece.limiters.obstacleAvoidanceDistanceFactor;
    setLimits(pp_motionProfile_type::SETPIECE, limits);

    pp_pid_params_struct_t pid;
    pid.X_P       = config.normal.pid.X_P;
    pid.X_I       = config.normal.pid.X_I;
    pid.X_D       = config.normal.pid.X_D;
    pid.Y_P       = config.normal.pid.Y_P;
    pid.Y_I       = config.normal.pid.Y_I;
    pid.Y_D       = config.normal.pid.Y_D;
    pid.XY_P      = config.normal.pid.XY_P;
    pid.XY_I      = config.normal.pid.XY_I;
    pid.XY_D      = config.normal.pid.XY_D;
    pid.PHI_P     = config.normal.pid.PHI_P;
    pid.PHI_I     = config.normal.pid.PHI_I;
    pid.PHI_D     = config.normal.pid.PHI_D;
    pid.maxI_XY   = config.normal.pid.maxI_XY;
    pid.maxI_Phi  = config.normal.pid.maxI_Phi;
    setPIDParams(pp_motionProfile_type::NORMAL, pid);

    pid.X_P       = config.setpiece.pid.X_P;
    pid.X_I       = config.setpiece.pid.X_I;
    pid.X_D       = config.setpiece.pid.X_D;
    pid.Y_P       = config.setpiece.pid.Y_P;
    pid.Y_I       = config.setpiece.pid.Y_I;
    pid.Y_D       = config.setpiece.pid.Y_D;
    pid.XY_P      = config.setpiece.pid.XY_P;
    pid.XY_I      = config.setpiece.pid.XY_I;
    pid.XY_D      = config.setpiece.pid.XY_D;
    pid.PHI_P     = config.setpiece.pid.PHI_P;
    pid.PHI_I     = config.setpiece.pid.PHI_I;
    pid.PHI_D     = config.setpiece.pid.PHI_D;
    pid.maxI_XY   = config.setpiece.pid.maxI_XY;
    pid.maxI_Phi  = config.setpiece.pid.maxI_Phi;
    setPIDParams(pp_motionProfile_type::SETPIECE, pid);

    pp_brake_params_struct_t brake_params;
    brake_params.deceleration    = config.normal.brake_deceleration;
    brake_params.gamma           = config.normal.brake_gamma;
    setBrakeParams(pp_motionProfile_type::NORMAL, brake_params);

    brake_params.deceleration    = config.setpiece.brake_deceleration;
    brake_params.gamma           = config.setpiece.brake_gamma;
    setBrakeParams(pp_motionProfile_type::SETPIECE, brake_params);

    pp_tokyo_drift_params_struct_t tokyoDriftParams;
    tokyoDriftParams.radius                = config.normal.tokyoDrift_radius;
    tokyoDriftParams.step_angle            = config.normal.tokyoDrift_step_angle;
    tokyoDriftParams.facing_target_tol     = config.normal.tokyoDrift_facing_target_tol;
    setTokyoDriftParams(pp_motionProfile_type::NORMAL, tokyoDriftParams);

    tokyoDriftParams.radius                = config.setpiece.tokyoDrift_radius;
    tokyoDriftParams.step_angle            = config.setpiece.tokyoDrift_step_angle;
    tokyoDriftParams.facing_target_tol     = config.setpiece.tokyoDrift_facing_target_tol;
    setTokyoDriftParams(pp_motionProfile_type::SETPIECE, tokyoDriftParams);
}

void cPathPlanningData::getLimits(pp_limiters_struct_t& limits)
{

    _mtx.lock();
    limits = _limits[_motionProfile];
    _mtx.unlock();

}
void cPathPlanningData::setLimits(const pp_motionProfile_type& motionProfile, const pp_limiters_struct_t& limits)
{
    _mtx.lock();
    _limits[motionProfile] = limits;
    _mtx.unlock();
}

void cPathPlanningData::getLinParams(pp_linear_params_struct_t& linParams)
{

    _mtx.lock();
    linParams = _linParams[_motionProfile];
    _mtx.unlock();

}
void cPathPlanningData::setLinParams(const pp_motionProfile_type& motionProfile, const pp_linear_params_struct_t& linParams)
{

    _mtx.lock();
    _linParams[motionProfile] = linParams;
    _mtx.unlock();

}

void cPathPlanningData::getPIDParams(pp_pid_params_struct_t& pidParams)
{

    _mtx.lock();
    pidParams = _pidParams[_motionProfile];
    _mtx.unlock();

}
void cPathPlanningData::setPIDParams(const pp_motionProfile_type& motionProfile, const pp_pid_params_struct_t& pidParams)
{

    _mtx.lock();
    _pidParams[motionProfile] = pidParams;
    _mtx.unlock();

}

void cPathPlanningData::getPFMParams(pp_pfm_params_struct_t& pfmParams)
{

    _mtx.lock();
    pfmParams = _pfmParams[_motionProfile];
    _mtx.unlock();

}
void cPathPlanningData::setPFMParams(const pp_motionProfile_type& motionProfile, const pp_pfm_params_struct_t& pfmParams)
{

    _mtx.lock();
    _pfmParams[motionProfile] = pfmParams;
    _mtx.unlock();

}

void cPathPlanningData::getBrakeParams(pp_brake_params_struct_t& brakeParams)
{

    _mtx.lock();
    brakeParams = _brakeParams[_motionProfile];
    _mtx.unlock();

}
void cPathPlanningData::setBrakeParams(const pp_motionProfile_type& motionProfile, const pp_brake_params_struct_t& brakeParams)
{

    _mtx.lock();
    _brakeParams[motionProfile] = brakeParams;
    _mtx.unlock();

}

void cPathPlanningData::getTokyoDriftParams(pp_tokyo_drift_params_struct_t& tokyoDriftParams)
{

    _mtx.lock();
    tokyoDriftParams = _tokyoDriftParams[_motionProfile];
    _mtx.unlock();

}
void cPathPlanningData::setTokyoDriftParams(const pp_motionProfile_type& motionProfile, const pp_tokyo_drift_params_struct_t& tokyoDriftParams)
{

    _mtx.lock();
    _tokyoDriftParams[motionProfile] = tokyoDriftParams;
    _mtx.unlock();

}

void cPathPlanningData::getObstacleAvoidanceIsEnabled(bool &isEnabled)
{
    _mtx.lock();
    isEnabled = _obstacleAvoidanceIsEnabled;
    _mtx.unlock();
}

void cPathPlanningData::setObstacleAvoidanceIsEnabled(const bool isEnabled)
{
    _mtx.lock();
    _obstacleAvoidanceIsEnabled = isEnabled;
    _mtx.unlock();
}


// Teamplay data
void cPathPlanningData::getTarget(Position2D& target)
{

    _mtx.lock();
    target = _targetPosition;
    _mtx.unlock();
}
void cPathPlanningData::setTarget(const Position2D& target)
{
    // Only adjust target when the target in inside the safe boundaries of the playing fields
    if(cEnvironmentField::getInstance().isPositionInArea(
            target.x, target.y, areaName::A_FIELD_SAFETY_BOUNDARIES, 0.0))
    {
        _mtx.lock();
        _targetPosition = target;
        cDiagnosticsAdapter::getInstance().setTarget(target.x, target.y, target.phi);
        _mtx.unlock();
        _logged_out_of_bounds = false;
    }
    else
    {
        // out of bounds; to avoid spam, only log once
        if(!_logged_out_of_bounds)
        {
            TRACE_INFO("Ignored out-of-bounds target x:%6.2f, y:%6.2f", target.x, target.y);
            _logged_out_of_bounds = true;
        }
    }
}

void cPathPlanningData::getTurnType(pp_algorithm_turn_type& turnType)
{

    _mtx.lock();
    turnType = _turnType;
    _mtx.unlock();

}
void cPathPlanningData::setTurnType(const pp_algorithm_turn_type& turnType)
{

    _mtx.lock();
    _turnType = turnType;
    _mtx.unlock();

}

void cPathPlanningData::getCoordType(pp_algorithm_coord_type& coordType)
{

    _mtx.lock();
    coordType = _coordType;
    _mtx.unlock();

}
void cPathPlanningData::setCoordType(const pp_algorithm_coord_type& coordType)
{

    _mtx.lock();
    _coordType = coordType;
    _mtx.unlock();

}

void cPathPlanningData::getPlotData(pp_plot_data_struct_t& plotData)
{

    _mtx.lock();
    plotData = _plotData;
    _mtx.unlock();

}
void cPathPlanningData::setPlotData(const pp_plot_data_struct_t& plotData)
{

    _mtx.lock();
    _plotData = plotData;
    _mtx.unlock();

}

void cPathPlanningData::getProjectedSpeedVectors(std::vector<linepoint2D>& projectedVectors)
{
    _mtx.lock();
    projectedVectors = _projectedSpeedVectors;
    _mtx.unlock();
}

void cPathPlanningData::setProjectedSpeedVectors(const std::vector<linepoint2D>& projectedVectors)
{
    _mtx.lock();
    _projectedSpeedVectors = projectedVectors;
    _mtx.unlock();
}
