 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cVelocityControlTypes.hpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Erik Kouters
 */

#ifndef CVELOCITYCONTROLTYPES_HPP_
#define CVELOCITYCONTROLTYPES_HPP_

#include <boost/assign/list_of.hpp>

// using Position2D and Velocity2D from FalconsCommon
#include "falconsCommon.hpp"
#include "area2D.hpp"
#include "polygon2D.hpp"

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
} vc_limiters_struct_t;

enum vc_algorithm_type
{
    moveWhileTurning,
    turnThenMove,
    moveThenTurn,
    moveAtSpeed,
    turn
};

enum vc_algorithm_turn_type
{
    normal,
    tokyo_drift
};

enum vc_algorithm_coord_type
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
} vc_data_struct_t;

typedef struct
{
    double pos;
    double vel;
    double acc;
    double jerk;
} vc_setpoint_output;

typedef struct
{
	double displacement;
	double velocity;
} vc_motor_data;

typedef struct
{
	vc_motor_data m1;
	vc_motor_data m2;
	vc_motor_data m3;
} vc_motors_data;

typedef struct
{
	double x;
	double y;
	double phi;
} vc_robot_displacement;

typedef struct
{
	double x;
	double y;
	double phi;
} vc_robot_velocity;

typedef struct
{
	vc_robot_displacement displacement;
	vc_robot_velocity velocity;
} vc_robot_data;

// Callback type to set algorithm type
//typedef boost::function<void(const vc_algorithm_type &vc_algoType)> CallbackType;

// Update function for VelocityControl
typedef boost::function<void()> iterateFunctionType;

typedef boost::function<void()> publishFeedbackFunctionType;

// publishSubtarget function for VelocityControl
typedef boost::function<void()> publishTargetFunctionType;

#endif /* CVELOCITYCONTROLTYPES_HPP_ */
