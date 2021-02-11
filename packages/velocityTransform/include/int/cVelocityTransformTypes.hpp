// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cVelocityTransformTypes.hpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Erik Kouters
 */

#ifndef CVELOCITYTRANSFORMTYPES_HPP_
#define CVELOCITYTRANSFORMTYPES_HPP_

#include <boost/assign/list_of.hpp>

// using Position2D and Velocity2D from FalconsCommon
#include "falconsCommon.hpp"
#include "area2D.hpp"
#include "polygon2D.hpp"

typedef struct
{
    Position2D pos;
    Velocity2D vel;
    Velocity2D acc;
    Velocity2D jerk;
} vt_data_struct_t;

typedef struct
{
    double pos;
    double vel;
    double acc;
    double jerk;
} vt_setpoint_output;

typedef struct
{
	double displacement;
	double velocity;
} vt_motor_data;

typedef struct
{
	vt_motor_data m1;
	vt_motor_data m2;
	vt_motor_data m3;
} vt_motors_data;

typedef struct
{
	double x;
	double y;
	double phi;
} vt_robot_displacement;

typedef struct
{
	double x;
	double y;
	double phi;
} vt_robot_velocity;

typedef struct
{
	vt_robot_displacement displacement;
	vt_robot_velocity velocity;
} vt_robot_data;

// Callback type to set algorithm type
//typedef boost::function<void(const vt_algorithm_type &vt_algoType)> CallbackType;

// Update function for VelocityTransform
typedef boost::function<void()> iterateFunctionType;

typedef boost::function<void()> publishFeedbackFunctionType;

// publishSubtarget function for VelocityTransform
typedef boost::function<void()> publishTargetFunctionType;

#endif /* CVELOCITYTRANSFORMTYPES_HPP_ */
