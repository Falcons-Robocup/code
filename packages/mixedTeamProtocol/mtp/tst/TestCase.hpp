// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#ifndef _INCLUDED_MIXEDTEAMPROTOCOL_TST_TESTCASE_HPP_
#define _INCLUDED_MIXEDTEAMPROTOCOL_TST_TESTCASE_HPP_

// system
#include <filesystem> // C++17

/* Include testframework */
#include "gtest/gtest.h"
#include "gmock/gmock.h"

// MTP headers
#include "int/AdapterRTDB.hpp"


using namespace ::testing;

namespace mtp {

class TestCase : public Test
{
public:
    TestCase()
    {
        return; // not needed anymore

        // communication workaround/trick
        // 1. either we need to instantiate comm (as process, or thread) per team
        // 2. or we need to make the data available for all agents

        // we choose for approach 2 and apply a symlink trick:
        // before each test, wipe the MTP rtdb databases for both teams and link then
        // also cleanup after the test

        // first determine the location of a helper script, which resides in the tst directory next to this file
        std::filesystem::path p(__FILE__);
        std::string command = std::string(p.parent_path()) + "/databaseLinkTrick.py";
        command += std::string(" ") + MTP_RTDB_STORAGE_PATH + "mixedteam_A";
        command += std::string(" ") + MTP_RTDB_STORAGE_PATH + "mixedteam_B";
        command += std::string(" ") + MTP_RTDB_STORAGE_PATH + "refbox_A";
        command += std::string(" ") + MTP_RTDB_STORAGE_PATH + "refbox_B";
        tprintf("running command: %s", command.c_str())
        system(command.c_str());
    };

    ~TestCase()
    {
        // TODO: maybe RTDB3 can provide a control API, for linking / wiping / deleting databases?
        // it seems the hard filesystem database removal is having negative side effects...
        return;

        std::string command = "rm -rf";
        command += std::string(" ") + MTP_RTDB_STORAGE_PATH + "mixedteam_A";
        command += std::string(" ") + MTP_RTDB_STORAGE_PATH + "mixedteam_B";
        command += std::string(" ") + MTP_RTDB_STORAGE_PATH + "refbox_A";
        command += std::string(" ") + MTP_RTDB_STORAGE_PATH + "refbox_B";
        if (command.size() > 20) // a little precaution in case for some reason the string would evaluate to "rm -rf /tmp" or something
        {
            tprintf("running command: %s", command.c_str())
            system(command.c_str());
        }
    };

};

} // end of namespace mtp

#endif
