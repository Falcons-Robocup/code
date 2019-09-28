 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * robotType.cpp
 *
 *  Created on: Aug 16, 2016
 *      Author: Tim Kouters
 */

#include "int/types/robot/robotType.hpp"

#include "FalconsCommon.h"

robotClass_t::robotClass_t()
{
    _robotID = 0;
    _timestamp = 0.0;
    _coordinates = coordinateType::FIELD_COORDS;
    _x = 0.0;
    _y = 0.0;
    _theta = 0.0;
    _vx = 0.0;
    _vy = 0.0;
    _vtheta = 0.0;
    _ballPossession = false;
}

robotClass_t::~robotClass_t()
/*
 * Chuck Norris doesn't dial the wrong number, you pick up the wrong phone
 */
{

}

void robotClass_t::setRobotID(const uint8_t robotID)
{
    _robotID = robotID;
}

void robotClass_t::setTimestamp(const double stamp)
{
    _timestamp = stamp;
}

void robotClass_t::setCoordinateType(const coordinateType coordinate)
{
    _coordinates = coordinate;
}

void robotClass_t::setCoordinates(const float x, const float y, const float theta)
{
    _x = x;
    _y = y;
    _theta = theta;
}

void robotClass_t::setVelocities(const float vx, const float vy, const float vtheta)
{
    _vx = vx;
    _vy = vy;
    _vtheta = vtheta;
}

void robotClass_t::setBallPossession(bool bp)
{
    _ballPossession = bp;
}

uint8_t robotClass_t::getRobotID() const
{
    return _robotID;
}

double robotClass_t::getTimestamp() const
{
    return _timestamp;
}

coordinateType robotClass_t::getCoordindateType() const
{
    return _coordinates;
}


float robotClass_t::getX() const
{
    return _x;
}

float robotClass_t::getY() const
{
    return _y;
}

float robotClass_t::getTheta() const
{
    return project_angle_0_2pi(_theta);
}

float robotClass_t::getVX() const
{
    return _vx;
}

float robotClass_t::getVY() const
{
    return _vy;
}

float robotClass_t::getVTheta() const
{
    return _vtheta;
}

bool robotClass_t::getBallPossession() const
{
    return _ballPossession;
}

