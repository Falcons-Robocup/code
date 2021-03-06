// Copyright 2018-2020 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2014-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef VIEWER_HPP
#define VIEWER_HPP

#include "preprocessor.hpp"
#include "ballDetection.hpp"
#include "obstacleDetection.hpp"
#include "cameraReceive.hpp"
#include "camSysReceive.hpp"
#include "determinePosition.hpp"
#include "linePointDetection.hpp"
#include "localization.hpp"
#include "robotFloor.hpp"
#include "configurator.hpp"

#include <iostream>

#include <mutex>

class viewer {
private:
    bool busy;
    char key;
    double period; // time between two updates (can be toggled with key 'u')
    int64 nextTick; // time when next viewers are updated
    int64 nextLogFileTick; // time when next line printed in logfile (through console)
    bool textOverlay;
    bool pause;
    bool help;
    uint16_t camHeight, camWidth, floorHeight, floorWidth;

    int viewMode;
    bool floorOverlay;
    bool dewarpOverlay;
    bool showCostField;
    bool ballView;
    bool ballFarView;
    bool cyanView;
    bool magentaView;
    // enum viewModeList { bgr=0, val, hue, sat, spiralLine, lineUnfiltered, lineErode, lineDilate, lineSel, ball, ballErode, ballDilate, cyan, magenta, obstacleSel, obstacleErode, obstacleDilate, last };
    enum viewModeList {
        bgr = 0,
        ball,
        ballErode,
        ballDilate,
        ballFar,
        ballFarErode,
        ballFarDilate,
        obstacleSel,
        obstacleErode,
        obstacleDilate,
        last,
        cyan,
        magenta,
        lineErode,
        lineDilate,
        lineSel
    };
    std::string selectWindow; // used for round (distorted) camera viewer
    std::string floorWindow; // used for rectangular (floor) viewer
    std::string whiteBallWindow; // used to show the white ball on the floor
    cv::Mat selectFrame, selectFrameExport; // use for round (distorted) viewer
    cv::Mat floorFrame, floorFrameExport; // used for rectangular (floor) viewer
    cv::Mat whiteBallFrame, whiteBallFrameExport; // used to find the white ball on the floor
    cv::Mat camFrameList[4];
    cv::Mat floorColorFrame; // used for rectangular (floor) viewer
    cv::Mat linePointsColorFrame; // used to copy the color from for the line points to the camera viewer window
    cv::Mat linePointsAllColorFrame; // used to copy the color from for all line points to the camera viewer window
    cv::Mat linePointsRejectColorFrame; // used to copy the color from for rejected line points to the camera viewer window
    std::vector<cv::Mat> planes;
    std::string viewModeText;
    std::vector<obstacleSt> cyans;
    std::vector<obstacleSt> magentas;
    std::vector<detPosSt> locations;

    std::mutex exportMutex;

    // pointers for access to other classes
    ballDetection *ballDet[4];
    ballDetection *ballFarDet[4];
    cameraReceive *camAnaRecv;
    camSysReceive *camSysRecv;
    configurator *conf;
    obstacleDetection *cyanDet;
    determinePosition *detPos;
    obstacleDetection *obstDet[4];
    preprocessor *prep;
    linePointDetection *linePoint;
    localization *loc;
    obstacleDetection *magentaDet;
    robotFloor *rFloor;
    Dewarper *dewarp[4];

    void roundViewerUpdate();
    void floorUpdate();
    void floorPrintText();

    void printStats();
    void printText();
    void drawLinePointsAll(cv::Scalar color);
    void drawObstacles();
    void drawFloorBalls(positionStDbl robot);
    void drawFloorBallsFar(positionStDbl robot);
    void drawFloorCyans(positionStDbl robot);
    void drawFloorMagentas(positionStDbl robot);
    void drawFloorObstacles(positionStDbl robot);
    void drawFloorLinePoints(positionStDbl pos, cv::Scalar color);
    void drawFloorRobot(positionStDbl pos, cv::Scalar color);

    void drawIsolineAzimuth(size_t cam, float az, cv::Vec3b color, int stride = 1);
    void drawIsolineElevation(size_t cam, float el, cv::Vec3b color, int stride = 1);
    void drawIsoline(cv::Mat &img, std::vector<cv::Point> const &pixels, cv::Vec3b color);

    void pixel2camFrame(int &x, int &y, int &cam);
    static void onMouse(int event, int x, int y, int flags, void* userdata);
    void handleClick(int x, int y);

public:
    viewer(ballDetection *ballDet[4], ballDetection *ballFarDet[4], cameraReceive *camAnaRecv,
            camSysReceive *camSysRecv, configurator *conf, obstacleDetection *cyanDet, determinePosition *detPos,
            linePointDetection *linePoint, localization *loc, obstacleDetection *magentaDet, obstacleDetection *obstDet[4],
            preprocessor *prep, robotFloor *rFloor, Dewarper *dewarp[4]);
    void update();
    cv::Mat getRoundFrame();
    cv::Mat getFloorFrame();
    cv::Mat getWhiteBallFrame();
    int getKey() {
        return key;
    }
    bool getPause() {
        return pause;
    }
    void setPause(int pause) {
        this->pause = pause;
    }
    void setViewModeBgr() {
        this->viewMode = bgr;
    }
    bool getBusy() {
        return busy;
    }
    void setBusy() {
        busy = true;
    }
};

#endif
