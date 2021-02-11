// Copyright 2019-2020 Coen Tempelaars (Falcons)
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
#include "FalconsRtDB2.hpp" // simulationTimeEnum

class AbstractConfigAdapter {
public:
    virtual ~AbstractConfigAdapter() {}

    virtual std::string getArbiter() const = 0;
    virtual int getSize(const TeamID) const = 0;
    virtual int getTickFrequency() const = 0;
    virtual int getStepSizeMs() const = 0;
};

#endif /* ABSTRACTCONFIGADAPTER_HPP_ */
