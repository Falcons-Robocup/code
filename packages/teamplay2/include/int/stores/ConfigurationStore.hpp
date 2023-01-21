// Copyright 2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * ConfigurationStore.hpp
 *
 *  Created on: Sep 27, 2016
 *      Author: Coen Tempelaars
 */

#ifndef CONFIGURATIONSTORE_HPP_
#define CONFIGURATIONSTORE_HPP_

#include "int/types/Configuration.hpp"

namespace teamplay
{

class ConfigurationStore {
public:
    static ConfigurationStore& getInstance()
    {
        static ConfigurationStore instance;
        return instance;
    }

    static Configuration& getConfiguration()
    {
        return getInstance()._configuration;
    }

private:
    ConfigurationStore();
    virtual ~ConfigurationStore();
    ConfigurationStore(ConfigurationStore const&); // Don't implement
    void operator= (ConfigurationStore const&); // Don't implement

    Configuration _configuration;

};


} /* namespace teamplay */

#endif /* CONFIGURATIONSTORE_HPP_ */
