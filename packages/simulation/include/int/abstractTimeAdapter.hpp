// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * abstractTimeAdapter.hpp
 *
 *  Created on: June 07, 2020
 *      Author: Erik Kouters
 */

#ifndef ABSTRACTTIMEADAPTER_HPP_
#define ABSTRACTTIMEADAPTER_HPP_

#include "FalconsRTDB.hpp" // for rtime

#include "teamID.hpp"
#include "robot.hpp"

class AbstractTimeAdapter {
public:
    virtual ~AbstractTimeAdapter() {}

    virtual void waitForHeartbeat() const = 0;
    virtual void publishSimulationTime (const rtime&) const = 0;
    virtual void publishSimulationHeartbeatDone() const = 0;
    virtual void waitForPutHeartbeatDone(TeamID teamID, RobotID robotID) const = 0;
};

#endif /* ABSTRACTTIMEADAPTER_HPP_ */

