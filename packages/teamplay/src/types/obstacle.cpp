 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * obstacle.cpp
 *
 *  Created on: Jan 5, 2018
 *      Author: Coen Tempelaars
 */

#include "falconsCommon.hpp"

#include "int/types/obstacle.hpp"

using namespace teamplay;


obstacle::obstacle()
    : _position()
    , _velocity()
{ }

obstacle::obstacle(const Position2D& pos)
: _position(pos)
, _velocity()
{ }

obstacle::obstacle(const Position2D& pos, const Velocity2D& vel)
    : _position(pos)
    , _velocity(vel)
{ }

obstacle::~obstacle()
{ }

Point2D obstacle::getLocation() const
{
    return Point2D(_position.x, _position.y);
}

Position2D obstacle::getPosition() const
{
    return _position;
}

Velocity2D obstacle::getVelocity() const
{
    return _velocity;
}

double obstacle::getDistanceTo (const Point2D& p) const
{
    return calc_distance(getLocation(), p);
}

void obstacle::setPosition(const Position2D& position)
{
    _position = position;
}

void obstacle::setVelocity(const Velocity2D& velocity)
{
    _velocity = velocity;
}

