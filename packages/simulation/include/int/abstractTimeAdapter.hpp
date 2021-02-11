// Copyright 2020 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * abstractTimeAdapter.hpp
 *
 *  Created on: June 07, 2020
 *      Author: Erik Kouters
 */

#ifndef ABSTRACTTIMEADAPTER_HPP_
#define ABSTRACTTIMEADAPTER_HPP_

#include "FalconsRtDB2.hpp" // for rtime

class AbstractTimeAdapter {
public:
    virtual ~AbstractTimeAdapter() {}

    virtual void waitForSimulationTick() const = 0;
    virtual void publishSimulationTick() const = 0;
    virtual void publishSimulationTime (const rtime&) const = 0;
};

#endif /* ABSTRACTTIMEADAPTER_HPP_ */

