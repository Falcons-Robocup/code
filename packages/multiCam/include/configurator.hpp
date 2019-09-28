 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2014-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef CONFIGURATOR_HPP
#define CONFIGURATOR_HPP

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <mutex>

// IMX219 camera is configured for x4 binning : 3280/4 = 820, 2464/4 = 616
// resolution is 820x616, but 820 does not divide by 8
// TODO: get from yaml
#define ROI_WIDTH 800
#define ROI_HEIGHT 608

#define CAMERA_HEIGHT 0.75 // meters above floor

typedef struct {
    uint16_t xBegin;
    uint16_t xEnd;
    uint16_t yBegin;
    uint16_t yEnd;
    uint16_t size;
} linePointSt;

typedef struct {
    double x;
    double y;
    double rz;
} positionStDbl;

typedef struct {
    int x;
    int y;
    int rz;
} positionStInt;

struct detPosSt {
    positionStDbl pos;
    bool goodEnough;
    double score;
    int lastActive; // last time (in frames) location was found
    int age; // life time in amount of frames
    int amountOnFloor;
    int amountOffFloor;
    int numberOfTries;
    bool operator<(const detPosSt& val) const {
        // sorting this struct is performed on score (lower is better) + lastActive (penalty) - age (improve)
        return (score + lastActive * 0.01f - age * 0.00001f) < (val.score + val.lastActive * 0.01f - val.age * 0.00001f);
    }
};

typedef struct {
    int xLeft;
    int xRight;
    int yTop;
    int yBottom;
} floorSizeSt;

typedef struct {
    cv::Mat window;
    int linePointsNumberMaximal;
    int linePointsNumberMinimal;
    int widthMin;
    int widthMax;
    int distance; // in pixels
    int distanceOffset; // in pixels
    int xCenter;
    float azimuth; // in radians
    int erodeSlider;
    cv::Mat erode;
    int dilateSlider;
    cv::Mat dilate;
    int score;
    int pixels;
    int pixelsMax;
    int pixelsNearby;
    int sunSpotWidth;
    int sunSpotHeight;
    int sunSpotRatio;
    int goalRearMask; // only used by robot1
    int goalRearDistance; // only used by robot1
    int goalRearAngle; // only used by robot1
    int goalRearWatchdog; // only used by robot1
} colStruct;

typedef struct {
    int distanceOffset;
    int width;
    int height;
    int threshold;
} possessionStruct;

typedef struct {
    int xStep;
    int yStep;
    int rzStep;
    int maxCount;
    int score;
} solverSt;

typedef struct {
    double xStep;
    double yStep;
    double rzStep;
    int maxCount;
    double score;
} solverStDbl;

typedef struct {
    double score;
    int amountOnFloor;
    int amountOffFloor;
    bool positionFound;
    double confidenceLevel;
} scoreStruct;

typedef struct {
    int keepWatchdog;
    double xDeltaKeepThreshold;
    double yDeltaKeepThreshold;
    double rzDeltaKeepThreshold;
    int newMinAge;
    double newThreshold;
    double angleToPixelRatio; // error = xDelta + yDelta + angleToPixelRatio * rzDelta
} goodEnoughSt;

typedef struct {
    int frequency;
    std::string ip;
    int port;
} multicastSt;

typedef struct {
    int x;
    int y;
    int rz;
    int rzBall;
    int rzObstacle;
} exportOffsetIntSt;

typedef struct {
    double x;
    double y;
    double rz;
    double rzBall;
    double rzObstacle;
} exportOffsetDblSt;

typedef struct {
    int leftFarAway;
    int leftCloseby;
    int rightFarAway;
    int rightCloseby;
} maskSt;

enum ballDetectionTypes {
    ballType = 0, ballFarType, cyanType, magentaType, obstacleType
};

#define COLOR_GRASS Scalar(0, 50 ,0)
#define COLOR_LINE Scalar(150, 150, 150)
#define COLOR_LINEPOINTS Scalar(255, 255, 255)
#define COLOR_LINEPOINTS_DARK Scalar(127, 127, 127)
#define COLOR_LINEPOINTS_OLD Scalar(127, 127 ,127)
#define COLOR_LINEPOINTS_INVALID Scalar(51, 51, 255);
#define COLOR_ROBOT Scalar(0, 0, 255)
#define COLOR_ROBOT_DARK Scalar(0, 0, 80)
#define COLOR_ROBOT_OLD Scalar(51, 153, 255);
#define COLOR_ROBOT_BALLPOSSESSION Scalar(0, 255, 255)
#define COLOR_OBSTACLE Scalar(0, 0 ,0)
#define COLOR_OBSTACLE_SMALL Scalar(0, 0 ,0)
#define COLOR_OBSTACLE_OLD Scalar(63, 63 ,63);
#define COLOR_FLOOR_TEXT Scalar(0, 190, 190)
#define COLOR_BALL Scalar(0, 255, 255)
#define COLOR_BALL_FAR Scalar(0, 127, 255)
#define COLOR_BALL_SMALL Scalar(0, 153, 153)
#define COLOR_BALL_FAR_SMALL Scalar(0, 105, 210)
#define COLOR_CYAN Scalar(255, 255, 0)
#define COLOR_CYAN_SMALL Scalar(139, 139, 0)
#define COLOR_MAGENTA Scalar(255, 0, 255)
#define COLOR_MAGENTA_SMALL Scalar(139, 0, 139)

class configurator {
private:
    std::string configWindow, colorWindowLine, colorWindowFloor, colorWindowBall, colorWindowBallFar, colorWindowPossession,
            colorWindowObstacle, colorWindowCyan, colorWindowMagenta, cameraWindow;

    cv::Point centerPoint;
    int cameraWidth, cameraHeight, cameraViewerWidth, cameraViewerHeight;
    bool remote; // used for remote viewer only
    positionStInt manual;
    bool manualMode;
    bool calibrateMode;
    bool guiEnabled;
    solverSt solver;
    int goalPostIn, goalPostOut;
    multicastSt multicast;
    goodEnoughSt goodEnough;
    double scoreThresHold;
    int locLatencyOffset;
    int ballLatencyOffset;
    int obstacleLatencyOffset;

    std::mutex lineStMutex, ballMutex, ballFarMutex, obstacleMutex, cyanMutex, magentaMutex;

    colStruct ball, ballFar, lineSt, obstacle, floorSt, cyan, magenta; // the color window ranges for the 4 targets
    possessionStruct possession;
    floorSizeSt floorSize;

    exportOffsetIntSt exportOffsetInt;
    exportOffsetDblSt exportOffsetDbl;
    int robot;
    int ballAzimuthDeg, ballFarAzimuthDeg, obstacleAzimuthDeg;

    int blurPixels;
    int pixelToAngle;
    int xHorizon;

    uint16_t cameraReceivePort;

    maskSt mask;

    void showTrackbars(int floorWidth, int floorHeight);
    void drawLineBGR();
    void drawFloorBGR();
    void drawBallBGR();
    void drawObstacleBGR();
    void drawCyanBGR();
    void drawMagentaBGR();

public:
    configurator(int robot);
    void init(int floorWidth, int floorHeight, bool guiEnabled);
    void update();

    // camera
    uint16_t getCameraReceivePort() {
        return cameraReceivePort;
    }

    /* Generic setters and getters */
    double getLocLatencyOffset() {
        return ((double) (locLatencyOffset) / 1000.0);
    }
    double getBallLatencyOffset() {
        return ((double) (ballLatencyOffset) / 1000.0);
    }
    double getObstacleLatencyOffset() {
        return ((double) (obstacleLatencyOffset) / 1000.0);
    }
    floorSizeSt getFloorSize() {
        return floorSize;
    }
    void setFloorSize(floorSizeSt value) {
        floorSize = value;
    }
    int getRobot() {
        return robot;
    }
    bool getRemote() {
        return remote;
    }
    void setRemote(bool value) {
        remote = value;
    }
    int getCameraWidth() {
        return cameraWidth;
    }
    int getCameraHeight() {
        return cameraHeight;
    }
    int getCameraViewerWidth() {
        return cameraViewerWidth;
    }
    int getCameraViewerHeight() {
        return cameraViewerHeight;
    }
    bool getGuiEnabled() {
        return guiEnabled;
    }
    exportOffsetDblSt getExportOffset() {
        exportOffsetDblSt exportOffsetDbl;
        exportOffsetDbl.x = (exportOffsetInt.x - 100) / 10.0; // the slider type is positive int, convert to range of -10.0 to + 10.0 in steps of 0.1
        exportOffsetDbl.y = (exportOffsetInt.y - 100) / 10.0;
        exportOffsetDbl.rz = (exportOffsetInt.rz - 100) / 10.0;
        exportOffsetDbl.rzBall = (exportOffsetInt.rzBall - 100) / 10.0;
        exportOffsetDbl.rzObstacle = (exportOffsetInt.rzObstacle - 100) / 10.0;
        return exportOffsetDbl;
    }
    int getGoalPostIn() {
        return goalPostIn;
    }
    int getGoalPostOut() {
        return goalPostOut;
    }
    multicastSt getMulticast() {
        return multicast;
    }
    goodEnoughSt getGoodEnough() {
        return goodEnough;
    }
    double getScoreThresHold() {
        return scoreThresHold;
    }
    positionStDbl getManual() {
        positionStDbl pos;
        pos.x = (double) manual.x;
        pos.y = (double) manual.y;
        pos.rz = (double) manual.rz;
        return pos;
    }
    void setManualMode(bool newValue) {
        manualMode = newValue;
    }
    bool getManualMode() {
        return manualMode;
    }
    void setCalibrateMode(bool newValue) {
        calibrateMode = newValue;
    }
    bool getCalibrateMode() {
        return calibrateMode;
    }
    int getBlurPixels() {
        if (calibrateMode) {
            return 2;
        } else {
            return blurPixels;
        }
    }
    int getPixelToAngle() {
        return pixelToAngle;
    }
    int getXHorizon() {
        return xHorizon;
    }

    solverStDbl getSolver() {
        solverStDbl retVal;
        retVal.xStep = (double) solver.xStep;
        retVal.yStep = (double) solver.yStep;
        retVal.rzStep = (double) solver.rzStep;
        retVal.maxCount = solver.maxCount;
        retVal.score = (double) solver.score / 100.0f;
        return retVal;
    }

    // color information for each type
    colStruct getLine() {
        lineStMutex.lock();
        colStruct tmp = lineSt;
        lineStMutex.unlock();
        return tmp;
    }
    colStruct getFloor() {
        return floorSt;
    }
    colStruct getBall(size_t type);
    colStruct getBallFar(size_t type);

    possessionStruct getPossession() {
        return possession;
    }

    maskSt getMask() {
        return mask;
    }

    // Below are internal functions but need to be public to for the static functions
    // that are required for the track bars.
    void updateLineTrackbar(int);
    void updateFloorTrackbar(int);
    void updateBallTrackbar(int);
    void updateBallFarTrackbar(int);
    void updateObstacleTrackbar(int);
    void updatePossessionTrackbar(int);
    void updateCyanTrackbar(int);
    void updateMagentaTrackbar(int);
    void updateLineErode(int newValue) {
        lineStMutex.lock();
        lineSt.erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
                cv::Point(newValue, newValue));
        lineStMutex.unlock();
    }
    void updateLineDilate(int newValue) {
        lineStMutex.lock();
        lineSt.dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
                cv::Point(newValue, newValue));
        lineStMutex.unlock();
    }
    void ballAzimuthUpdate(int newValue) {
        ball.azimuth = CV_PI * newValue / 180.0;
    }
    void ballFarAzimuthUpdate(int newValue) {
        ball.azimuth = CV_PI * newValue / 180.0;
    }
    void updateBallErode(int newValue) {
        ballMutex.lock();
        ball.erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
                cv::Point(newValue, newValue));
        ballMutex.unlock();
    }
    void updateBallFarErode(int newValue) {
        ballFarMutex.lock();
        ballFar.erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
                cv::Point(newValue, newValue));
        ballFarMutex.unlock();
    }
    void updateBallDilate(int newValue) {
        ballMutex.lock();
        ball.dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
                cv::Point(newValue, newValue));
        ballMutex.unlock();
    }
    void updateBallFarDilate(int newValue) {
        ballFarMutex.lock();
        ballFar.dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
                cv::Point(newValue, newValue));
        ballFarMutex.unlock();
    }
    void updateObstacleErode(int newValue) {
        obstacleMutex.lock();
        obstacle.erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
                cv::Point(newValue, newValue));
        obstacleMutex.unlock();
    }
    void updateObstacleDilate(int newValue) {
        obstacleMutex.lock();
        obstacle.dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
                cv::Point(newValue, newValue));
        obstacleMutex.unlock();
    }
    void obstacleAzimuthUpdate(int newValue) {
        obstacle.azimuth = CV_PI * newValue / 180.0;
    }
    void updateCyanErode(int newValue) {
        cyanMutex.lock();
        cyan.erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
                cv::Point(newValue, newValue));
        cyanMutex.unlock();
    }
    void updateCyanDilate(int newValue) {
        cyanMutex.lock();
        cyan.dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
                cv::Point(newValue, newValue));
        cyanMutex.unlock();
    }
    void updateMagentaErode(int newValue) {
        magentaMutex.lock();
        magenta.erode = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
                cv::Point(newValue, newValue));
        magentaMutex.unlock();
    }
    void updateMagentaDilate(int newValue) {
        magentaMutex.lock();
        magenta.dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(newValue * 2 + 1, newValue * 2 + 1),
                cv::Point(newValue, newValue));
        magentaMutex.unlock();
    }

    static void cbUpdateLineTrackbar(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->updateLineTrackbar(newValue);
    }

    static void cbUpdateFloorTrackbar(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->updateFloorTrackbar(newValue);
    }

    static void cbUpdateBallTrackbar(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->updateBallTrackbar(newValue);
    }

    static void cbUpdateBallFarTrackbar(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->updateBallFarTrackbar(newValue);
    }

    static void cbUpdatePossessionTrackbar(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->updatePossessionTrackbar(newValue);
    }

    static void cbUpdateCyanTrackbar(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->updateCyanTrackbar(newValue);
    }

    static void cbUpdateMagentaTrackbar(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->updateMagentaTrackbar(newValue);
    }

    static void cbUpdateLineErodeTrackbar(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->updateLineErode(newValue);
    }

    static void cbUpdateLineDilateTrackbar(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->updateLineDilate(newValue);
    }

    static void ballAzimuthTrackbarUpdate(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->ballAzimuthUpdate(newValue);
    }

    static void ballFarAzimuthTrackbarUpdate(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->ballFarAzimuthUpdate(newValue);
    }

    static void cbUpdateBallErodeTrackbar(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->updateBallErode(newValue);
    }

    static void cbUpdateBallFarErodeTrackbar(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->updateBallFarErode(newValue);
    }

    static void cbUpdateBallDilateTrackbar(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->updateBallDilate(newValue);
    }

    static void cbUpdateBallFarDilateTrackbar(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->updateBallFarDilate(newValue);
    }

    static void cbUpdateObstacleTrackbar(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->updateObstacleTrackbar(newValue);
    }

    static void cbUpdateObstacleErodeTrackbar(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->updateObstacleErode(newValue);
    }

    static void cbUpdateObstacleDilateTrackbar(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->updateObstacleDilate(newValue);
    }

    static void obstacleAzimuthTrackbarUpdate(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->obstacleAzimuthUpdate(newValue);
    }

    static void cbUpdateCyanErodeTrackbar(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->updateCyanErode(newValue);
    }

    static void cbUpdateCyanDilateTrackbar(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->updateCyanDilate(newValue);
    }

    static void cbUpdateMagentaErodeTrackbar(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->updateMagentaErode(newValue);
    }

    static void cbUpdateMagentaDilateTrackbar(int newValue, void * object) {
        configurator *conf = (configurator*) object;
        conf->updateMagentaDilate(newValue);
    }

};

#endif
