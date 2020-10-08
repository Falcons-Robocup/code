 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * robotLocationType.cpp
 *
 *  Created on: Oct 25, 2016
 *      Author: Tim Kouters
 */

#include "types/robotLocationType.hpp"

robotLocationType::robotLocationType()
{
	_x = 0.0;
	_y = 0.0;
	_theta = 0.0;
	_confidence = 0.0;
	_fps = 0.0;
	_linePoints = 0;
	_age = 0.0;
	_lastActive = 0.0;
}

robotLocationType::~robotLocationType()
{

}

void robotLocationType::setX(const float x)
{
	_x = x;
}

void robotLocationType::setY(const float y)
{
	_y = y;
}

void robotLocationType::setTheta(const float theta)
{
	_theta = theta;
}

void robotLocationType::setConfidence(const float confidence)
{
	_confidence = confidence;
}

void robotLocationType::setFPS(const float fps)
{
	_fps = fps;
}

void robotLocationType::setLinePoints(const int linePoints)
{
	_linePoints = linePoints;
}

void robotLocationType::setAge(const float age)
{
	_age = age;
}

void robotLocationType::setLastActive(const float lastActive)
{
	_lastActive = lastActive;
}

float robotLocationType::getX() const
{
	return _x;
}

float robotLocationType::getY() const
{
	return _y;
}

float robotLocationType::getTheta() const
{
	return _theta;
}

float robotLocationType::getConfidence() const
{
	return _confidence;
}

float robotLocationType::getFPS() const
{
	return _fps;
}

int robotLocationType::getLinePoints() const
{
	return _linePoints;
}

float robotLocationType::getAge() const
{
	return _age;
}

float robotLocationType::lastActive() const
{
	return _lastActive;
}
