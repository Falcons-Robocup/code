 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * robotVelocityType.cpp
 *
 *  Created on: Nov 8, 2018
 *      Author: Erik Kouters
 */

#include "int/types/robot/robotVelocityType.hpp"

robotVelocityClass_t::robotVelocityClass_t()
{
	_coordinate = coordinateType::FIELD_COORDS;
	_displacementSource = displacementType::INVALID;
	_timestamp = 0.0;
	_vx = 0.0;
	_vy = 0.0;
	_vtheta = 0.0;
}

robotVelocityClass_t::~robotVelocityClass_t()
/*
 * Chuck Norris can cut a knife with butter
 */
{

}

void robotVelocityClass_t::setID(const uniqueObjectID identifier)
{
	_identifier = identifier;
}

void robotVelocityClass_t::setCoordinateType(const coordinateType coordinates)
{
	_coordinate = coordinates;
}

void robotVelocityClass_t::setDisplacementSource(const displacementType displacementSource)
{
	_displacementSource = displacementSource;
}

void robotVelocityClass_t::setTimestamp(const double timestamp)
{
	_timestamp = timestamp;
}

void robotVelocityClass_t::setDeltaVelocity(const float vx, const float vy, const float vtheta)
{
	_vx = vx;
	_vy = vy;
	_vtheta = vtheta;
}


uniqueObjectID robotVelocityClass_t::getID() const
{
	return _identifier;
}

coordinateType robotVelocityClass_t::getCoordinateType() const
{
	return _coordinate;
}

displacementType robotVelocityClass_t::getDisplacementSource() const
{
	return _displacementSource;
}

double robotVelocityClass_t::getTimestamp() const
{
	return _timestamp;
}

float robotVelocityClass_t::getvX() const
{
	return _vx;
}

float robotVelocityClass_t::getvY() const
{
	return _vy;
}

float robotVelocityClass_t::getvTheta() const
{
	return _vtheta;
}

