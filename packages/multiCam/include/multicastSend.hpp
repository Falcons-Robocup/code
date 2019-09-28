 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2017-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef MULTICAST_SEND
#define MULTICAST_SEND

#include "ballDetection.hpp"
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
    ballDetection *cyanDet;
    determinePosition *detPos;
    linePointDetection *linePoint;
    localization *loc;
    ballDetection *magentaDet;
    ballDetection *obstDet[4];
    preprocessor *prep;
    robotFloor *rFloor;

    std::vector<observer*> vecObservers; // to make it possible to write diagnostics to RTDB

public:

    multicastSend(ballDetection *ballDet[4], ballDetection *ballFarDet[4], cameraReceive *camAnaRecv,
            camSysReceive *camSysRecv, configurator *conf, ballDetection *cyanDet, determinePosition *detPos,
            linePointDetection *linePoint, localization *loc, ballDetection *magentaDet, ballDetection *obstDet[4],
            preprocessor *prep, robotFloor *rFloor);
    void send();
    void printNetworkInterface(char *buff, int flags);
    void stats();
    void locList();
    void goodEnoughLoc();
    void floorLinePoints();
    void ballList(size_t type);
    
    void attach(observer *observer);
    void detach(observer *observer);
    
};

#endif
