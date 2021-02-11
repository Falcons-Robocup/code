// Copyright 2020 Erik Kouters (Falcons)
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
    virtual void waitForSimulationTick() const;
    virtual void publishSimulationTick() const;
    virtual void publishSimulationTime (const rtime&) const;
};

#endif /* RTDBTIMEADAPTER_HPP_ */

