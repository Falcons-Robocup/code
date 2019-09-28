 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef CAM_GRAB_RECEIVE_HPP
#define CAM_GRAB_RECEIVE_HPP

#include "camPacket.hpp"
#include "raspiControl.hpp"

#include <mutex>
#include <stdlib.h>

// data is wrapped around in buffer, so the first element does not need to be the oldest
struct camTimeSt {
    uint64_t cam[4][32];
};

class camGrabReceive {
private:
    int multReceiveFd;
    camPacketT rxPacket;
    uint8_t rxPacketCntExpected[4];
    bool rxPacketCntFirstCheck[4];

    raspiControl *raspiCtrl;

    std::mutex cam0GrabExportMutex;
    std::mutex cam1GrabExportMutex;
    std::mutex cam2GrabExportMutex;
    std::mutex cam3GrabExportMutex;

    camTimeSt camTime;

    size_t cam0Index, cam1Index, cam2Index, cam3Index;

public:
    camGrabReceive(raspiControl *raspiCtrl);
    void receive(); // blocking call, run in separate thread

    camTimeSt getCamTime();
};

#endif
