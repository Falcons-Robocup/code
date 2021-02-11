// Copyright 2019 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "camGrabReceive.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <iostream>
#include <net/if.h>
#include <pwd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

using namespace std;

camGrabReceive::camGrabReceive(raspiControl *raspiCtrl) {
	this->raspiCtrl = raspiCtrl;

    // create a normal UDP socket
    if ((multReceiveFd = socket( AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0) {
        printf("ERROR     : cannot create UDP socket for receiving from raspiGrabber, message: %s\n", strerror(errno));
        fflush(stdout);
        exit(EXIT_FAILURE);
    }

    // allow multiple sockets to use the same port
    int reuse = 1;
    if (setsockopt(multReceiveFd, SOL_SOCKET, SO_REUSEADDR, (char *) &reuse, sizeof(reuse)) < 0) {
        printf(
                "ERROR     : cannot configure port for multiple UTP sockets for receiving from raspiGrabber, message: %s\n",
                strerror(errno));
        fflush(stdout);
        exit(EXIT_FAILURE);
    }

    // fill in the local address structure
    struct sockaddr_in localAddr;
    memset((void *) &localAddr, 0, sizeof(localAddr));
    localAddr.sin_family = AF_INET; // Internet
    localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    localAddr.sin_port = htons(11111); // raspi grabber synchronization port
    // bind to the local address / port
    if (::bind(multReceiveFd, (struct sockaddr *) &localAddr, sizeof(localAddr)) < 0) {
        printf("ERROR     : cannot bind to UDP socket for receiving from raspiGrabber, message: %s\n", strerror(errno));
        fflush(stdout);
        exit(EXIT_FAILURE);
    }

    // join the multicast group
    struct ip_mreq mreq;
    memset((void *) &mreq, 0, sizeof(mreq)); // set all to zero
    mreq.imr_multiaddr.s_addr = inet_addr("224.16.16.16"); // multicast group on local network
    // first try IP robot configuration
    mreq.imr_interface.s_addr = inet_addr("10.0.0.1");
    if (setsockopt(multReceiveFd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &mreq, sizeof(mreq)) < 0) {
        // fallback
        mreq.imr_interface.s_addr = htonl(INADDR_ANY);
        if (setsockopt(multReceiveFd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *) &mreq, sizeof(mreq)) < 0) {
            printf("ERROR     : cannot join the multicast group from receiving from raspiGrabber, %s\n",
                    strerror(errno));
            fflush(stdout);
            exit(EXIT_FAILURE);
        }
    }

    printf("INFO      : raspiGrabber receive multicast group address %s port %u\n", inet_ntoa(mreq.imr_multiaddr),
            ntohs(localAddr.sin_port));

    // initialize the output buffers
    cam0GrabExportMutex.lock();
    for (size_t ii = 0; ii < 32; ii++) {
        camTime.cam[0][ii] = 0;
    }
    cam0GrabExportMutex.unlock();

    cam1GrabExportMutex.lock();
    for (size_t ii = 0; ii < 32; ii++) {
        camTime.cam[1][ii] = 0;
    }
    cam1GrabExportMutex.unlock();

    cam2GrabExportMutex.lock();
    for (size_t ii = 0; ii < 32; ii++) {
        camTime.cam[2][ii] = 0;
    }
    cam2GrabExportMutex.unlock();

    cam3GrabExportMutex.lock();
    for (size_t ii = 0; ii < 32; ii++) {
        camTime.cam[3][ii] = 0;
    }
    cam3GrabExportMutex.unlock();

    // initilize the pointers for the output buffers
    cam0Index = 0;
    cam1Index = 0;
    cam2Index = 0;
    cam3Index = 0;

}

void camGrabReceive::receive() {
    while (1) {
        // block until data received
        struct sockaddr_in fromAddr;
        socklen_t fromAddrlen = sizeof(fromAddr);
        ssize_t nBytes = recvfrom(multReceiveFd, &rxPacket, sizeof(camPacketT), 0, (struct sockaddr *) &fromAddr,
                &fromAddrlen);
        if (nBytes < 0) {
            printf("ERROR     : reading from raspiGrabber socket, message: %s\n", strerror(errno));
            close(multReceiveFd);
            exit(EXIT_FAILURE);
        }

        bool valid = true;
        if (rxPacket.size != nBytes) {
            printf("ERROR     : received %zd bytes, but expected %u bytes from raspyGrabber\n", nBytes, rxPacket.size);
            valid = false;
        } else {
            //printf("INFO      : received %u bytes from raspiGrabber %s:%u\n", rxPacket.size, inet_ntoa(fromAddr.sin_addr),
            //            ntohs(fromAddr.sin_port));
        }

        size_t camIndex = rxPacket.id >> 6; // highest 2 bits contain the camera location (camera id)

        if (valid && (rxPacket.cnt != rxPacketCntExpected[camIndex])) {
            if (rxPacketCntFirstCheck[camIndex]) {
                rxPacketCntFirstCheck[camIndex] = false;
                rxPacketCntExpected[camIndex] = rxPacket.cnt + 1; // set the expected for the next received camera packet
            } else {
                printf(
                        "WARNING   : cam %zu received camera packet counter %3u from raspiGrabber, but expected packet counter %3u\n",
                        camIndex, rxPacket.cnt, rxPacketCntExpected[camIndex]);
                rxPacketCntExpected[camIndex] = rxPacket.cnt + 1;
                valid = false;
            }
        } else {
            rxPacketCntExpected[camIndex]++;
        }

        if (valid) {
            struct timeval tv;
            gettimeofday(&tv, NULL);
            uint64_t localTime = tv.tv_sec * 1000000 + tv.tv_usec;

            if (camIndex == 0) {
                //printf("INFO      : received raspiGrabber synchronization packet from camera 0 time %6.3f seconds\n", deltaTime / 1000000.0);
                cam0GrabExportMutex.lock();
                camTime.cam[0][cam0Index] = localTime;
                cam0GrabExportMutex.unlock();
                // send the frame counter of cam0 to all camera's (including cam0)
                // which is used by cam1 to cam3 to synchronize the frame counter with cam0
                uint32_t cam0FrameCounter = rxPacket.pl.u32[0];
                raspiCtrl->cam0FrameCntSend(cam0FrameCounter);
                if (cam0Index >= 31) {
                    cam0Index = 0;
                } else {
                    cam0Index++;
                }
            } else if (camIndex == 1) {
                //printf("INFO      : received raspiGrabber synchronization packet from camera 1 time %6.3f seconds\n", deltaTime / 1000000.0);
                cam1GrabExportMutex.lock();
                camTime.cam[1][cam1Index] = localTime;
                cam1GrabExportMutex.unlock();
                if (cam1Index >= 31) {
                    cam1Index = 0;
                } else {
                    cam1Index++;
                }
            } else if (camIndex == 2) {
                //printf("INFO      : received raspiGrabber synchronization packet from camera 2 time %6.3f seconds\n", deltaTime / 1000000.0);
                cam2GrabExportMutex.lock();
                camTime.cam[2][cam2Index] = localTime;
                cam2GrabExportMutex.unlock();
                if (cam2Index >= 31) {
                    cam2Index = 0;
                } else {
                    cam2Index++;
                }
            } else {
                //printf("INFO      : received raspiGrabber synchronization packet from camera 3 time %6.3f seconds\n", deltaTime / 1000000.0);
                cam3GrabExportMutex.lock();
                camTime.cam[3][cam3Index] = localTime;
                cam3GrabExportMutex.unlock();
                if (cam3Index >= 31) {
                    cam3Index = 0;
                } else {
                    cam3Index++;
                }
            }
        } // if (valid
    } // while ( 1
} // void camGrabReceive::receive()

camTimeSt camGrabReceive::getCamTime() {
    camTimeSt tmp;
    // keep the lock time as short as possible so the receive time is most accurate
    cam0GrabExportMutex.lock();
    for (size_t ii = 0; ii < 32; ii++) {
        tmp.cam[0][ii] = camTime.cam[0][ii];
    }
    cam0GrabExportMutex.unlock();

    cam1GrabExportMutex.lock();
    for (size_t ii = 0; ii < 32; ii++) {
        tmp.cam[1][ii] = camTime.cam[1][ii];
    }
    cam1GrabExportMutex.unlock();

    cam2GrabExportMutex.lock();
    for (size_t ii = 0; ii < 32; ii++) {
        tmp.cam[2][ii] = camTime.cam[2][ii];
    }
    cam2GrabExportMutex.unlock();

    cam3GrabExportMutex.lock();
    for (size_t ii = 0; ii < 32; ii++) {
        tmp.cam[3][ii] = camTime.cam[3][ii];
    }
    cam3GrabExportMutex.unlock();
    return tmp;
}

