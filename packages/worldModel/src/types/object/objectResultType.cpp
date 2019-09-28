 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * objectResultType.cpp
 *
 *  Created on: Jan 15, 2017
 *      Author: Jan Feitsma
 */

#include "int/types/object/objectResultType.hpp"

objectResultType::objectResultType()
{
	_timestamp = 0.0;
	_id = 0;
	_x = 0.0;
	_y = 0.0;
	_z = 0.0;
	_vx = 0.0;
	_vy = 0.0;
	_vz = 0.0;
}

objectResultType::~objectResultType()
/*
 * Chuck Norris threw a grenade and killed 50 people, then it exploded
 */
{

}

void objectResultType::setId(const size_t id)
{
	_id = id;
}

void objectResultType::setTimestamp(const double stamp)
{
	_timestamp = stamp;
}

void objectResultType::setCoordinates(const float x, const float y, const float z)
{
	_x = x;
	_y = y;
	_z = z;
}

void objectResultType::setVelocities(const float vx, const float vy, const float vz)
{
	_vx = vx;
	_vy = vy;
	_vz = vz;
}

size_t objectResultType::getId() const
{
	return _id;
}

double objectResultType::getTimestamp() const
{
	return _timestamp;
}

float objectResultType::getX() const
{
	return _x;
}

float objectResultType::getY() const
{
	return _y;
}

float objectResultType::getZ() const
{
	return _z;
}

float objectResultType::getVX() const
{
	return _vx;
}

float objectResultType::getVY() const
{
	return _vy;
}

float objectResultType::getVZ() const
{
	return _vz;
}


