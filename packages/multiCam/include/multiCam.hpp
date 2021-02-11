// Copyright 2018-2019 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2016-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef MULTICAM_HPP
#define MULTICAM_HPP

#include "ballDetection.hpp"
#include "obstacleDetection.hpp"
#include "cameraReceive.hpp"
#include "camSysReceive.hpp"
#include "configurator.hpp"
#include "determinePosition.hpp"
#include "dewarp.hpp"
#include "linePointDetection.hpp"
#include "localization.hpp"
#include "multicastSend.hpp"
#include "observer.hpp"
#include "preprocessor.hpp"
#include "robotFloor.hpp"
#include "viewer.hpp"

#include <iostream>
#include <thread>

class multiCamLibrary
{
private:
    configurator *conf;
    obstacleDetection *cyanDet;
    obstacleDetection *magentaDet;
    ballDetection *ballDet[4];
    ballDetection *ballFarDet[4];
    cameraReceive *camAnaRecv;
    camSysReceive *camSysRecv;
    determinePosition *detPos;
    Dewarper *dewarp[4];
    obstacleDetection *obstDet[4];
    linePointDetection *linePoint;
    localization *loc;
    multicastSend *multSend;
    preprocessor *prep;
    robotFloor *rFloor;
    viewer *view;

    cv::VideoCapture capture;
    int frameCurrent;
    bool guiEnabled;
    cv::Mat frameRaw;
    int robotId;
    bool useCamera;

    double previousSendTime;

    std::thread camAnaRecvThread, camSysRecvThread, ballDetThread[4], ballFarDetThread[4], cyanDetThread, magentaDetThread, ballPosThread, locThread, obstDetThread[4], viewerThread;

public:
    multiCamLibrary( int robotIdArg, bool guiEnabled );
    bool update( );
    void attach( observer *obs );
};

#endif
