// Copyright 2020-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBTimeAdapter.hpp
 *
 *  Created on: June 07, 2020
 *      Author: Erik Kouters
 */

#ifndef RTDBTIMEADAPTER_HPP_
#define RTDBTIMEADAPTER_HPP_

#include "abstractTimeAdapter.hpp"

class RTDBTimeAdapter : public AbstractTimeAdapter {
public:
    virtual void waitForHeartbeat() const;
    virtual void publishSimulationTime (const rtime&) const;
    virtual void publishSimulationHeartbeatDone() const;
    virtual void waitForPutHeartbeatDone(TeamID teamID, RobotID robotID) const;
};

#endif /* RTDBTIMEADAPTER_HPP_ */

