// Copyright 2016 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * configurationStore.hpp
 *
 *  Created on: Sep 27, 2016
 *      Author: Coen Tempelaars
 */

#ifndef CONFIGURATIONSTORE_HPP_
#define CONFIGURATIONSTORE_HPP_

#include "int/types/configuration.hpp"

namespace teamplay
{

class configurationStore {
public:
    static configurationStore& getInstance()
    {
        static configurationStore instance;
        return instance;
    }

    static configuration& getConfiguration()
    {
        return getInstance()._configuration;
    }

private:
    configurationStore();
    virtual ~configurationStore();
    configurationStore(configurationStore const&); // Don't implement
    void operator= (configurationStore const&); // Don't implement

    configuration _configuration;

};


} /* namespace teamplay */

#endif /* CONFIGURATIONSTORE_HPP_ */
