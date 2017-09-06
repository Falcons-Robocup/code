 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * ballPositionType.cpp
 *
 *  Created on: Oct 25, 2016
 *      Author: Tim Kouters
 */

#include "types/ballPositionType.hpp"

ballPositionType::ballPositionType()
{
	_angle = 0.0;
	_radius = 0.0;
	_z = 0.0;
	_confidence = 0.0;
}

ballPositionType::~ballPositionType()
{

}


void ballPositionType::setAngle(const float angle)
{
	_angle = angle;
}

void ballPositionType::setRadius(const float radius)
{
	_radius = radius;
}

void ballPositionType::setZ(const float z)
{
	_z = z;
}

void ballPositionType::setConfidence(const float confidence)
{
	_confidence = confidence;
}

void ballPositionType::setBallType(size_t type)
{
	_type = type;
}



float ballPositionType::getAngle() const
{
	return _angle;
}

float ballPositionType::getRadius() const
{
	return _radius;
}

float ballPositionType::getZ() const
{
	return _z;
}

float ballPositionType::getConfidence() const
{
	return _confidence;
}

size_t ballPositionType::getBallType() const
{
	return _type;
}

