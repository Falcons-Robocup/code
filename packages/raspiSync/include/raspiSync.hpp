// Copyright 2019 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef RASPI_SYNC_HPP
#define RASPI_SYNC_HPP

#include "camGrabReceive.hpp"
#include "raspiControl.hpp"

#include <cstdint>
#include <iostream>
#include <thread>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class raspiSync {
private:

    raspiControl *raspiCtrl; // used to send pixel offset to the camera's

    camGrabReceive *camGrabRecv;

    struct timespec nextWakeTime; // to align "exactly" on the 500ms of the clock

    std::thread camGrabRecvThread;

    cv::Mat timeFrame;

    cv::Scalar cam0Color;
    cv::Scalar cam1Color;
    cv::Scalar cam2Color;
    cv::Scalar cam3Color;
    cv::Scalar referenceColor;
    cv::Scalar markerColor[4];

    int pixelsOffset[4];
    int pixelsOffsetPrev[4];
    int pixelsOffsetIntegrator[4];

public:
    raspiSync();
    ~raspiSync();
    bool update();
};

#endif
