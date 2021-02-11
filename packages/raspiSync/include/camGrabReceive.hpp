// Copyright 2019 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
