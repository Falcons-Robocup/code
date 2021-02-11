// Copyright 2015-2016 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cPositionTypes.hpp
 *
 *  Created on: Sep 15, 2015
 *      Author: Ivo Matthijssen
 */

#ifndef CPOSITIONTYPES_HPP_
#define CPOSITIONTYPES_HPP_

#include <vector>

typedef struct
{
	float x;
	float y;
} position2D_struct_t;

typedef std::vector<position2D_struct_t> position2D_list_t;

typedef struct
{
	float x;
	float y;
	float vx;
	float vy;
} vector2D_struct_t;

typedef std::vector<vector2D_struct_t> vector2D_list_t;

typedef struct
{
	float x;
	float y;
	float theta;
} position3D_struct_t;

typedef std::vector<position3D_struct_t> position3D_list_t;

typedef struct
{
	int robotID;
	float x;
	float y;
	float theta;
	float vx;
	float vy;
	float vtheta;
} vector3D_struct_t;

typedef std::vector<vector3D_struct_t> vector3D_list_t;

typedef struct
{
	position2D_struct_t upper_left;
	position2D_struct_t lower_right;
} area2D_struct_t;


#endif /* CPOSITIONTYPES_HPP_ */
