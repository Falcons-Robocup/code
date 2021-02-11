// Copyright 2018-2019 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2017-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef MULTICAST_SEND
#define MULTICAST_SEND

#include "ballDetection.hpp"
#include "obstacleDetection.hpp"
#include "cameraReceive.hpp"
#include "camSysReceive.hpp"
#include "configurator.hpp"
#include "determinePosition.hpp"
#include "linePointDetection.hpp"
#include "localization.hpp"
#include "multicastCommon.hpp"
#include "preprocessor.hpp"
#include "robotFloor.hpp"
#include "observer.hpp"

#include <arpa/inet.h>
#include <net/if.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

class multicastSend {
private:
    int fd;
    uint8_t packetCnt;
    uint16_t packetSize;
    struct sockaddr_in toAddr;
    packetT sendBuf;

    // pointers for access to other classes
    ballDetection *ballDet[4];
    ballDetection *ballFarDet[4];
    cameraReceive *camAnaRecv;
    camSysReceive *camSysRecv;
    configurator *conf;
    obstacleDetection *cyanDet;
    determinePosition *detPos;
    linePointDetection *linePoint;
    localization *loc;
    obstacleDetection *magentaDet;
    obstacleDetection *obstDet[4];
    preprocessor *prep;
    robotFloor *rFloor;

    std::vector<observer*> vecObservers; // to make it possible to write diagnostics to RTDB

public:

    multicastSend(ballDetection *ballDet[4], ballDetection *ballFarDet[4], cameraReceive *camAnaRecv,
            camSysReceive *camSysRecv, configurator *conf, obstacleDetection *cyanDet, determinePosition *detPos,
            linePointDetection *linePoint, localization *loc, obstacleDetection *magentaDet, obstacleDetection *obstDet[4],
            preprocessor *prep, robotFloor *rFloor);
    void send();
    void printNetworkInterface(char *buff, int flags);
    void stats();
    void locList();
    void goodEnoughLoc();
    void floorLinePoints();
    void objectList(size_t type);
    
    void attach(observer *observer);
    void detach(observer *observer);
    
};

#endif
