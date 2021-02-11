// Copyright 2019 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBMotionAdapter.hpp
 *
 *  Created on: Feb 12, 2019
 *      Author: Coen Tempelaars
 */

#ifndef RTDBMOTIONADAPTER_HPP_
#define RTDBMOTIONADAPTER_HPP_

#include "abstractMotionAdapter.hpp"

class RTDBMotionAdapter : public AbstractMotionAdapter {
public:
    virtual Velocity2D getVelocity (const TeamID&, const RobotID&) const;
    virtual Kicker getKickerData (const TeamID&, const RobotID&) const;
    virtual bool hasBallHandlersEnabled (const TeamID&, const RobotID&) const;
};

#endif /* RTDBMOTIONADAPTER_HPP_ */
