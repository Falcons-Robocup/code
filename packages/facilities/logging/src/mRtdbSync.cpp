// Copyright 2018 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * mRtdbSync.cpp
 *
 * Executable controlling syncing of RTDB storages.
 *
 *  Created on: Aug 19, 2018
 *      Author: Jan Feitsma
 */


#include "ext/cDbSync.hpp"
#include <boost/lexical_cast.hpp>


int main(int argc, char **argv)
{
    int frequency = 10;
    if (argc > 1)
    {
        frequency = boost::lexical_cast<int>(argv[1]);
    }
    sleep(2);
    cDbSync dbSync(frequency);
    dbSync.run();
    
    return 0;
}

