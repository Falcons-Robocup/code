 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * robot.hpp
 *
 *  Created on: Nov 27, 2016
 *      Author: Coen Tempelaars
 */

#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include <string>
#include <vector>
#include "boost/optional.hpp"

#include "position2d.hpp"
#include "vector2d.hpp"
#include "int/types/fieldDimensions.hpp"
#include "int/types/role.hpp"

namespace teamplay
{

typedef unsigned char robotNumber;

class robot {
public:
    robot();
    robot(const robotNumber);
    robot(const robotNumber, const treeEnum&, const Position2D&, const Velocity2D&);
    virtual ~robot();

    virtual bool hasBall() const;
    virtual bool isInArea(const fieldArea&) const;
    virtual bool isOwnRobot() const;

    virtual robotNumber getNumber() const;
    virtual treeEnum getRole() const;
    virtual boost::optional<treeEnum> getAssistantRole() const;
    virtual Point2D getLocation() const;
    virtual Position2D getPosition() const;
    virtual Velocity2D getVelocity() const;
    virtual double getDistanceTo (const Point2D&) const;

    virtual void setNumber(const robotNumber);
    virtual void setRole(const treeEnum&);
    virtual void setPosition(const Position2D&);
    virtual void setVelocity(const Velocity2D&);

    virtual void claimsBallPossession();
    virtual void losesBallPossession();

    virtual void setOwnRobot();
    virtual void setNotOwnRobot();

    virtual std::string str() const;

private:
    robotNumber _number;
    role _role;
    Position2D _position;
    Velocity2D _velocity;
    bool _hasBall;
    bool _isOwnRobot;
};

typedef std::vector<robot> robots;

} /* namespace teamplay */

#endif /* ROBOT_HPP_ */
