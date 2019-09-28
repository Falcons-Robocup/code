// Copyright 2018-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "camSysReceive.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <iostream>
#include <mutex>
#include <net/if.h>
#include <pwd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <thread>
#include <unistd.h> // getopt

int main(int argc, char** argv) {
    int opt = 0;
    while ((opt = getopt(argc, argv, "h")) != -1) {
        switch (opt) {
        case 'h':
            printf("INFO    : no arguments\n");
            exit(EXIT_SUCCESS);
            break;
        }
    }

    int32_t processId = getpid();
    printf("INFO      : cam %u system process ID %d\n", 0, processId);


    camSysReceive *camSysRecv = new camSysReceive();

    // start thread that collects system data from multiple camera's
    std::thread camSysRecvThread = std::thread(&camSysReceive::receive, camSysRecv);

    while (1) {
        for (size_t camIndex = 0; camIndex < 4; camIndex++) {
            camSystemSt cs = camSysRecv->getCamSystem(camIndex);
            if (cs.updated) {
                hhMmSsSt applUp = camSysRecv->secondsToHhMmSs(cs.applUptime);
                hhMmSsSt cpuUp = camSysRecv->secondsToHhMmSs(cs.cpuUptime);

                printf(
                        "INFO      : cam %zu system cpu %2u gpu %2u cmos %3d load %4.2f v_err %u f_err %u t_err %u md5sum %08x cpu %02u:%02u:%02u appl %02u:%02u:%02u\n",
                        camIndex, cs.cpuTemp, cs.gpuTemp, cs.camSensTemp, cs.cpuLoad, cs.underVoltage,
                        cs.frequencyCapped, cs.throttling, cs.md5sumShort, cpuUp.hours, cpuUp.minutes, cpuUp.seconds,
                        applUp.hours, applUp.minutes, applUp.seconds);
                if (cs.underVoltageOccured) {
                    printf("ERROR     : cam %zu system UNDER VOLTAGE OCCURED\n", camIndex);
                }

                if (cs.frequencyCapped) {
                    printf("WARNING   : cam %zu system ARM FREQUENCY CAPPED HAS OCCURED\n", camIndex);
                }

                if (cs.throttlingOccured) {
                    printf("WARNING   : cam %zu system THROTTLING HAS OCCURED\n", camIndex);
                }
            } // if (cs.updated
        } // for (size_t camIndex)
        usleep(1000);
    } // while (1
    return 0;
}
