// Copyright 2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * abstractMotionAdapter.hpp
 *
 *  Created on: Jan 16, 2019
 *      Author: Coen Tempelaars
 */

#ifndef ABSTRACTMOTIONADAPTER_HPP_
#define ABSTRACTMOTIONADAPTER_HPP_

#include "robot.hpp"
#include "teamID.hpp"

class AbstractMotionAdapter {
public:
    virtual ~AbstractMotionAdapter() {}

    virtual Velocity2D getVelocity (const TeamID&, const RobotID&) const = 0;
    virtual Kicker getKickerData (const TeamID&, const RobotID&) const = 0;
    virtual bool hasBallHandlersEnabled (const TeamID&, const RobotID&) const = 0;
};

#endif /* ABSTRACTMOTIONADAPTER_HPP_ */
