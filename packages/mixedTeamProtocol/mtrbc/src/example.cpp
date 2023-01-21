// Copyright 2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
#include <iostream>
#include "RtDB2.h"
#include "comm.hpp"

using namespace std;

int main()
{
    // setup and run comm to receive refbox commands using own agent id
    // (in this example own agent id equals 1)
    RtDB2Context ctx = RtDB2Context::Builder(1, RtDB2ProcessType::comm)
                           .withConfigFileName("mtrbc/rtdb2_refbox.xml") // describes the refbox network
                           .withNetwork("refboxclient")                  // refboxclient network is receive-only
                           .build();
    Comm comm(ctx);
    comm.start();

    RtDB2 rtdb(ctx, 0); // accessor to database of mixed-team refbox client (mtrbc)
    std::string last_command("");
    while (true)
    {
        std::string command;
        std::string target;
        bool success = (rtdb.get("COMMAND", &command) == RTDB2_SUCCESS) &&
                       (rtdb.get("TARGETTEAM", &target) == RTDB2_SUCCESS);
        if (success && last_command.compare(command) != 0)
        {
            // received new command
            std::cout << "Received command: " << command << "; target team: " << target << std::endl;
            last_command = command;
        }
        usleep(10000);
    }

    return 0;
}
