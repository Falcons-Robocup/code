// Copyright 2016-2018 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
