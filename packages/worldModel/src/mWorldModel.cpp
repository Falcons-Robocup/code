// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * mainRTDB.cpp
 *
 *  Created on: Jul 8, 2018
 *      Author: Jan Feitsma
 */

#include "int/cWorldModel.hpp"

#include "tracing.hpp"


int main(int argc, char **argv)
{
    try
    {
        INIT_TRACE;
        
        // setup worldmodel
        cWorldModel w;
        if (argc > 1 && std::string(argv[1]) == std::string("--inplay"))
        {
            w.enableInplayOverrule();
        }
        
        /* They see me rollin', they hatin' */
        w.run();
    }
    catch(std::exception &e)
    {
        TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

