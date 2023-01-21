// Copyright 2019-2021 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * abstractConfigAdapter.hpp
 *
 *  Created on: Aug 17, 2019
 *      Author: Coen Tempelaars
 */

#ifndef ABSTRACTCONFIGADAPTER_HPP_
#define ABSTRACTCONFIGADAPTER_HPP_

#include "teamID.hpp"
#include <string>
#include "FalconsRTDB.hpp" // simulationTimeEnum

class AbstractConfigAdapter {
public:
    virtual ~AbstractConfigAdapter() {}

    virtual float getTickFrequency() const = 0;
    virtual float getSimulationSpeedupFactor() const = 0;
    virtual std::string getTickFinishRtdbKey() const = 0;
};

#endif /* ABSTRACTCONFIGADAPTER_HPP_ */
