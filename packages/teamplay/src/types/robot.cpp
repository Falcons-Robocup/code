 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * robot.cpp
 *
 *  Created on: Nov 27, 2016
 *      Author: Coen Tempelaars
 */

#include <iostream>
#include <stdexcept>
#include <sstream>
#include <string>

#include "int/types/robot.hpp"

using namespace teamplay;


robot::robot()
    : number(0)
    , role(treeEnum::R_ROBOT_STOP)
    , position()
    , velocity()
{ }

robot::robot(const int _number)
: number(_number)
, role(treeEnum::R_ROBOT_STOP)
, position()
, velocity()
{ }

robot::robot(const int _number, const treeEnum& _role, const Position2D& _pos, const Velocity2D& _vel)
    : number(_number)
    , role(_role)
    , position(_pos)
    , velocity(_vel)
{
    if((role < treeEnum::ATTACKER_MAIN) || (role > treeEnum::R_ROBOT_STOP))
    {
        std::ostringstream msg;
        msg << "Error: attempt to construct a robot with an illegal role";
        std::cerr << msg.str() << std::endl;
        throw std::runtime_error(msg.str());
    }
}

robot::~robot()
{ }

int robot::getNumber() const
{
    return number;
}

treeEnum robot::getRole() const
{
    return role;
}

Point2D robot::getLocation() const
{
    return Point2D(position.x, position.y);
}

Position2D robot::getPosition() const
{
    return position;
}

Velocity2D robot::getVelocity() const
{
    return velocity;
}

void robot::setNumber(const int _number)
{
    number = _number;
}

void robot::setRole(const treeEnum& _role)
{
    role = _role;
}

void robot::setPosition(const Position2D& _position)
{
    position = _position;
}

void robot::setVelocity(const Velocity2D& _velocity)
{
    velocity = _velocity;
}
