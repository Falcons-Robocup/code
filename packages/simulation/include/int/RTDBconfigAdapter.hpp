// Copyright 2019-2020 Coen Tempelaars (Falcons)
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
    virtual std::string getArbiter() const;
    virtual int getSize(const TeamID) const;
    virtual int getTickFrequency() const;
    virtual int getStepSizeMs() const;

private:
    ConfigRTDBAdapter<T_CONFIG_SIMULATION>* _configAdapter;
};

#endif /* RTDBCONFIGADAPTER_HPP_ */
