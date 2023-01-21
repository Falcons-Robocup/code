// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * Robot.hpp
 *
 *  Created on: Nov 27, 2016
 *      Author: Coen Tempelaars
 */

#ifndef ROBOT_HPP_
#define ROBOT_HPP_

#include <string>
#include <vector>
#include "boost/optional.hpp"

#include "velocity2d.hpp"
#include "pose2d.hpp"
#include "int/types/FieldDimensions.hpp"
#include "int/types/Role.hpp"

namespace teamplay
{

typedef unsigned char RobotNumber;

class Robot {
public:
    Robot();
    Robot(const RobotNumber);
    Robot(const RobotNumber, const RoleEnum&, const geometry::Pose2D&, const geometry::Velocity2D&);
    virtual ~Robot();

    virtual bool operator== (const Robot&) const;
    virtual bool operator!= (const Robot&) const;
    virtual bool operator< (const Robot&) const;

    virtual bool hasBall() const;
    virtual bool isInArea(const FieldArea&) const;
    virtual bool isOwnRobot() const;

    virtual RobotNumber getNumber() const;
    virtual RoleEnum getRole() const;
    virtual boost::optional<RoleEnum> getAssistantRole() const;
    virtual Point2D getLocation() const;
    virtual geometry::Pose2D getPosition() const;
    virtual geometry::Velocity2D getVelocity() const;
    virtual double getDistanceTo (const Point2D&) const;

    virtual void setNumber(const RobotNumber);
    virtual void setRole(const RoleEnum&);
    virtual void setPosition(const geometry::Pose2D&);
    virtual void setVelocity(const geometry::Velocity2D&);

    virtual void claimsBallPossession();
    virtual void losesBallPossession();

    virtual void setOwnRobot();
    virtual void setNotOwnRobot();

    virtual std::string str() const;

private:
    RobotNumber _number;
    Role _role;
    geometry::Pose2D _position;
    geometry::Velocity2D _velocity;
    bool _hasBall;
    bool _isOwnRobot;
};

typedef std::vector<Robot> robots;

} /* namespace teamplay */

#endif /* ROBOT_HPP_ */
