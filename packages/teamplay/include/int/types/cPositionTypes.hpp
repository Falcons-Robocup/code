 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
