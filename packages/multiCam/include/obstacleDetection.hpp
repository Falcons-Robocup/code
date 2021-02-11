// Copyright 2018-2019 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Jan Feitsma, december 2019

#ifndef OBSTACLEDETECTION_HPP
#define OBSTACLEDETECTION_HPP

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core/core.hpp"

#include "cameraReceive.hpp"
#include "configurator.hpp"
#include "dewarp.hpp"
#include "preprocessor.hpp"
#include "observer.hpp"

#include <mutex>

#include "object.hpp" // commonality between balls and obstacles
typedef objectSt obstacleSt;


class obstacleDetection {

private:
    cameraReceive *camRecv;
    configurator *conf;
    Dewarper *dewarp;
    preprocessor *prep;

    size_t camIndex;
    uint16_t width, height;
    size_t printCount;

    size_t type; // re-used for cyan and magenta detection, type is used to distinguish

    cv::Mat dilateFrame;
    cv::Mat erodeFrame;
    cv::Mat inRangeFrame; // used to search the balls
    cv::Mat possessionFrame;

    size_t possessionPixels;

    std::vector<ssize_t> closestPixels;
    std::vector<ssize_t> closestGroups;

    std::vector<obstacleSt> positions;
    std::vector<obstacleSt> positionsRemoteViewer;
    std::vector<linePointSt> obstaclePointList; // used by viewer

    std::mutex positionsRemoteViewerExportMutex;
    std::mutex obstaclePointListExportMutex;
    std::mutex exportMutex;

    bool busy;

    // List of observers for Obstacle position notifications
    std::vector<observer*> vecObservers; // only for Ros
    void notifyNewPos(); // only for Ros
    void findClosestLineGroups();

public:

    obstacleDetection(cameraReceive *camRecv, configurator *conf, Dewarper *dewarp, preprocessor *prep, size_t type,
            size_t cam);
    void keepGoing();
    void update();

    std::vector<linePointSt> getObstaclePoints();
    std::vector<obstacleSt> getPositions();
    std::vector<obstacleSt> getAndClearPositionsRemoteViewer();
    std::vector<obstacleSt> getPositionsExport();
    std::vector<ssize_t> getClosestPixels();
    std::vector<ssize_t> getClosestGroups();
    cv::Mat getPossessionFrame();
    cv::Mat getDilateFrame();
    cv::Mat getErodeFrame();
    cv::Mat getInRangeFrame();
    size_t getAmount();

    bool getBusy() {
        return busy;
    }
    void setBusy() {
        busy = true;
    }

    void attach(observer *observer);
    void detach(observer *observer);
};
#endif
