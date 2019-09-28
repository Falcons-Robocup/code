 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
