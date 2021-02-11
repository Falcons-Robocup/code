// Copyright 2016 Coen Tempelaars (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * configurationStore.cpp
 *
 *  Created on: Sep 27, 2016
 *      Author: Coen Tempelaars
 */

#include "int/stores/configurationStore.hpp"

using namespace teamplay;

configurationStore::configurationStore()
{
    _configuration = configuration();
}

configurationStore::~configurationStore()
{
}
