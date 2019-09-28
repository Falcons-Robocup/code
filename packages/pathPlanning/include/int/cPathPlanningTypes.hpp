 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cPathPlanningTypes.hpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Tim Kouters
 */

#ifndef CPATHPLANNINGTYPES_HPP_
#define CPATHPLANNINGTYPES_HPP_

#include <boost/assign/list_of.hpp>

// using Position2D and Velocity2D from FalconsCommon
#include "FalconsCommon.h"
#include "area2D.hpp"
#include "polygon2D.hpp"

//circumference = 2*pi*r = pi*d = 3.14*0.46 = 1.4444 m
//1.4444m == 2pi
//1 rad = 1.4444 / 2pi = 0.2298833998 m for 1 rad.
// Used in: cTeamplayAdapter::targetCallback
const float radPerSecToMeterPerSec = 0.2298833998;

// motion profile groups
enum class pp_motionProfile_type
{
    NORMAL = 0,
    SETPIECE,

    SIZE_OF_ENUM
};
static std::map<std::string, pp_motionProfile_type> motionProfileEnumMapping = boost::assign::map_list_of
        ("normal",  pp_motionProfile_type::NORMAL)
        ("setpiece", pp_motionProfile_type::SETPIECE)
        ;

// internal obstacle type
enum class pp_obstacle_type
{
	INVALID,
	WORLDMODEL_OBSTACLES,
	WORLDMODEL_TEAMMEMBERS,
	PROJECTED,
	FORBIDDEN_AREA
};

typedef struct
{
    Position2D location;
    Velocity2D velocity;
    double a_i;
    double b_i;
    enum pp_obstacle_type type;
} pp_obstacle_struct_t;

typedef struct
{
        float maxVelXY;
        float maxVelXY_withBall;
        float maxVelPhi;
        float maxVelPhi_withBall;
        float maxAccXY;
        float maxAccPhi;
        float tolerationXY;
        float tolerationPhi;
        float relativeSpeedFactorX;
        float relativeSpeedFactorY;
        float relativeSpeedFactorPhi;
        float obstacleAvoidanceScalingFactor;
        float obstacleAvoidanceDistanceFactor;
} pp_limiters_struct_t;

typedef struct
{
   double X_P;
   double X_I;
   double X_D;
   double Y_P;
   double Y_I;
   double Y_D;
   double XY_P;
   double XY_I;
   double XY_D;
   double PHI_P;
   double PHI_I;
   double PHI_D;
   double maxI_XY;
   double maxI_Phi;
} pp_pid_params_struct_t;

typedef struct
{
   double gainXY;
   double gainPhi;
} pp_linear_params_struct_t;

typedef struct
{
   double dist_lim;
   double force_max;
   double force_min;
   double expo_rep;
   double mult_rep;
   double gain_rep;
   double gain_attr;
} pp_pfm_params_struct_t;

typedef struct
{
    double deceleration;
    double gamma;
} pp_brake_params_struct_t;

typedef struct
{
    double radius;
    double step_angle;
    double facing_target_tol;
} pp_tokyo_drift_params_struct_t;

typedef struct
{
	Position2D pos_t;//Target
	Position2D pos_a;//Actual
	Velocity2D vel_t;
	Velocity2D vel_a;
    double x_pid_out;
    double x_pid_p;
    double x_pid_i;
    double x_pid_d;
    double y_pid_out;
    double y_pid_p;
    double y_pid_i;
    double y_pid_d;
    double phi_pid_out;
    double phi_pid_p;
    double phi_pid_i;
    double phi_pid_d;
    double pos_reached;
    double tokyo_drift;
    double error_x;
    double error_y;
    double jerk_x;
    double jerk_y;
    double acc_x;
    double acc_y;
    double vel_x;
    double vel_y;
    double pos_x;
    double pos_y;
} pp_plot_data_struct_t;

enum pp_algorithm_type
{
    moveWhileTurning,
    turnThenMove,
    moveThenTurn,
    moveAtSpeed,
    turn
};

enum pp_algorithm_turn_type
{
    normal,
    tokyo_drift
};

enum pp_algorithm_coord_type
{
    robot_coord,
    field_coord
};

typedef struct
{
    Position2D pos;
    Velocity2D vel;
    Velocity2D acc;
    Velocity2D jerk;
} pp_data_struct_t;

typedef struct
{
    double pos;
    double vel;
    double acc;
    double jerk;
} pp_setpoint_output;

// Callback type to set algorithm type
//typedef boost::function<void(const pp_algorithm_type &pp_algoType)> CallbackType;

// Update function for PathPlanning
typedef boost::function<void()> iterateFunctionType;

// publishSpeed function for PathPlanning
typedef boost::function<void(const Velocity2D& vel_rcs)> publishSpeedFunctionType;

// publishSubtarget function for PathPlanning
typedef boost::function<void(const double& x, const double& y)> publishSubtargetFunctionType;

// publishObstacles function for PathPlanning
typedef boost::function<void(const std::vector<polygon2D>& forbiddenAreas, std::vector<linepoint2D>& projectedSpeedVectors)> publishObstaclesFunctionType;

// publishPlotData function for PathPlanning
typedef boost::function<void(const pp_plot_data_struct_t& plotData)> publishPlotDataFunctionType;

#endif /* CPATHPLANNINGTYPES_HPP_ */
