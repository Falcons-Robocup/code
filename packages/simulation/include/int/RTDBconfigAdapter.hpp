// Copyright 2019-2021 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * RTDBConfigAdapter.hpp
 *
 *  Created on: Aug 17, 2019
 *      Author: Coen Tempelaars
 */

#ifndef RTDBCONFIGADAPTER_HPP_
#define RTDBCONFIGADAPTER_HPP_

#include "abstractConfigAdapter.hpp"

#include "ConfigRTDBAdapter.hpp"

class RTDBConfigAdapter : public AbstractConfigAdapter {
public:
    RTDBConfigAdapter();
    ~RTDBConfigAdapter();
    virtual float getTickFrequency() const;
    virtual float getSimulationSpeedupFactor() const;
    virtual std::string getTickFinishRtdbKey() const;

private:
    ConfigRTDBAdapter<T_CONFIG_EXECUTION>* _configAdapter;
};

#endif /* RTDBCONFIGADAPTER_HPP_ */
