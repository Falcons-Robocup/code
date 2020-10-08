 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2014-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "configurator.hpp"

#include "falconsCommonDirs.hpp"

#include <unistd.h>
#include <pwd.h>

using namespace cv;
using namespace std;

configurator::configurator(int robot) {
    guiEnabled = false;
    ballAzimuthDeg = 0;
    obstacleAzimuthDeg = 0;

    lineStMutex.lock();
    ballMutex.lock();
    obstacleMutex.lock();
    magentaMutex.lock();
    cyanMutex.lock();

    std::string configFile = pathToConfig() + "/multiCam.yaml";
    FileStorage fs(configFile, FileStorage::READ);

    FileNode global = fs["global"];
    remote = false; // default not used for remote viewer
    cameraWidth = (int) global["cameraWidth"]; // 824 pixels
    cameraHeight = (int) global["cameraHeight"]; // 616 pixels
    cameraViewerWidth = cameraWidth + cameraHeight / 2; // viewer uses half size of the camera images 2*412 + 308
    cameraViewerHeight = cameraViewerWidth;

    multicast.frequency = (int) (global["multicastFrequency"]);
    multicast.ip = (string) (global["multicastIp"]);
    multicast.port = (int) (global["multicastPort"]);
    cameraReceivePort = (int) (global)["cameraReceivePort"];

    goodEnough.angleToPixelRatio = (double) (global["goodEnoughAngleToPixelRatio"]);
    goodEnough.xDeltaKeepThreshold = (double) (global["goodEnoughXDeltaKeepThreshold"]);
    goodEnough.yDeltaKeepThreshold = (double) (global["goodEnoughYDeltaKeepThreshold"]);
    goodEnough.rzDeltaKeepThreshold = (int) (global["goodEnoughRzDeltaKeepThreshold"]);
    goodEnough.keepWatchdog = (int) (global["goodEnoughKeepWatchdog"]);
    goodEnough.newMinAge = (int) (global["goodEnoughNewMinAge"]);
    goodEnough.newThreshold = (double) (global["goodEnoughNewThreshold"]);
    scoreThresHold = (double) (global["scoreThresHold"]);
    locLatencyOffset = (int) (global["locLatencyOffset"]);
    ballLatencyOffset = (int) (global["ballLatencyOffset"]);
    obstacleLatencyOffset = (int) (global["obstacleLatencyOffset"]);

    manual.x = 0; // set in configurator::init with values from robot floor class
    manual.y = 0; // set in configurator::init with values from robot floor class
    manual.rz = 0; // set in configurator::init

    this->manualMode = false;
    this->calibrateMode = false;
    solver.xStep = 150;
    solver.yStep = 150;
    solver.rzStep = 30; // when using compass, otherwise 100
    solver.maxCount = 400;
    solver.score = 20;
    blurPixels = 80; // increased for larger field (more error in far away pixels)
    pixelToAngle = (int) global["pixelToAngle"];
    xHorizon = (int) global["xHorizon"];

    this->robot = robot;

    FileNode specific;
    if (robot == 1) {
        specific = fs["robot1"];
    } else if (robot == 2) {
        specific = fs["robot2"];
    } else if (robot == 3) {
        specific = fs["robot3"];
    } else if (robot == 4) {
        specific = fs["robot4"];
    } else if (robot == 5) {
        specific = fs["robot5"];
    } else if (robot == 6) {
        specific = fs["robot6"];
    } else if (robot == 7) {
        specific = fs["robot7"];
    } else if (robot == 8) {
        specific = fs["robot8"];
    } else {
        std::cerr << "multiCam: Robot " << robot << " has no configuration" << std::endl;
        exit(1);
    }

    lineSt.distance = (int) (specific["linePointDistance"]);
    lineSt.distanceOffset = (int) (specific["linePointDistanceOffset"]);
    possession.distanceOffset = (int) (specific["ballPossesionDistanceOffset"]);
    ball.distanceOffset = 0;
    ballFar.distanceOffset = 0;
    obstacle.distanceOffset = (int) (specific["obstacleDistanceOffset"]);
    magenta.distanceOffset = 0;
    cyan.distanceOffset = 0;

    mask.leftFarAway = (int) (specific["maskLeftFarAway"]);
    mask.leftCloseby = (int) (specific["maskLeftCloseby"]);
    mask.rightFarAway = (int) (specific["maskRightFarAway"]);
    mask.rightCloseby = (int) (specific["maskRightCloseby"]);

    ballAzimuthDeg = (int) (specific["ballAzimuth"]);
    ballFarAzimuthDeg = (int) (specific["ballFarAzimuth"]);
    ball.azimuth = CV_PI * ballAzimuthDeg / 180.0;
    ballFar.azimuth = CV_PI * ballFarAzimuthDeg / 180.0;
    obstacleAzimuthDeg = (int) (specific["obstacleAzimuth"]);
    obstacle.azimuth = CV_PI * obstacleAzimuthDeg / 180.0;
    magenta.azimuth = CV_PI * (int) (specific["magentaAzimuth"]) / 180.0;
    cyan.azimuth = CV_PI * (int) (specific["cyanAzimuth"]) / 180.0;

    exportOffsetInt.x = (int) (specific["xExportOffset"]);
    exportOffsetInt.y = (int) (specific["yExportOffset"]);
    exportOffsetInt.rz = (int) (specific["rzExportOffset"]);
    exportOffsetInt.rzBall = (int) (specific["rzBallExportOffset"]); // use this ball rz correction also for cyan and magenta
    exportOffsetInt.rzObstacle = (int) (specific["rzObstacleExportOffset"]);

    // project additional goal posts on the field
    goalPostIn = (int) (specific["goalPostIn"]);
    goalPostOut = (int) (specific["goalPostOut"]);

    // these colors could also be different for each robot, but probably not
    // create color window for white lines
    FileNode line = fs["line"];
    lineSt.linePointsNumberMaximal = (int) (line["linePointsNumberMaximal"]); // maximal amount of line points used as input search
    lineSt.linePointsNumberMinimal = (int) (line["linePointsNumberMinimal"]); // minimal amount of line points that have been used
    lineSt.widthMin = (int) (line["widthMin"]); // amount of points to be classified as line
    lineSt.widthMax = (int) (line["widthMax"]); // maximal amount of line points used for search
    lineSt.erodeSlider = (int) (line["erodeSlider"]); // used find sun spot (large wide area on field because of bright sunlight)
    lineSt.dilateSlider = (int) (line["dilateSlider"]); // used find sun spot
    if (robot == 1) {
        lineSt.goalRearMask = (int) (line["goalRearMask"]); // amount of pixels blocked by robot1 to prevent issues with the goal net
    } else {
        lineSt.goalRearMask = 0; // not used by other robots
    }
    lineSt.goalRearDistance = (int) (line["goalRearDistance"]); // maximal pixel distance from robot to goal line where mask is allowed
    lineSt.goalRearAngle = (int) (line["goalRearAngle"]); // maximal pixel distance from robot to goal line where mask is allowed
    lineSt.goalRearWatchdog = (int) (line["goalRearWatchdog"]); // watchdog (in frames) removing mask when no new good enough position has been provided

    // create color window for yellow ball
    FileNode ballConf = fs["ball"];
    ball.widthMin = 0; // not used
    ball.widthMax = 0; // not used
    ball.erodeSlider = (int) (ballConf["erodeSlider"]);
    ball.dilateSlider = (int) (ballConf["dilateSlider"]);
    ball.pixels = (int) (ballConf["pixels"]); // minimal required pixels detected by camera : countNonZero(inRangeFrame(rect))
    ball.pixelsNearby = (int) (ballConf["pixelsNearby"]); // minimal required pixels detected by camera : countNonZero(inRangeFrame(rect))
    ball.xCenter = (int) (ballConf["xCenter"]); // correct for ball diameter in millimeters

    // create color window for yellow far ball
    FileNode ballFarConf = fs["ballFar"];
    ballFar.widthMin = 0; // not used
    ballFar.widthMax = 0; // not used
    ballFar.erodeSlider = (int) (ballFarConf["erodeSlider"]);
    ballFar.dilateSlider = (int) (ballFarConf["dilateSlider"]);
    ballFar.pixels = (int) (ballFarConf["pixels"]); // minimal required pixels detected by camera : countNonZero(inRangeFrame(rect))
    ballFar.pixelsMax = (int) (ballFarConf["pixelsMax"]); // maximal pixels detected by camera : countNonZero(inRangeFrame(rect))
    ballFar.pixelsNearby = (int) (ballFarConf["pixelsNearby"]); // minimal required pixels detected by camera : countNonZero(inRangeFrame(rect))
    ballFar.xCenter = (int) (ballFarConf["xCenter"]); // correct for far away ball diameter in millimeters

    // create color window for the ball possession
    FileNode possessionConf = fs["possession"];
    // the possession.yOffset is set at the robot specific section
    possession.width = (int) (possessionConf["width"]);
    possession.height = (int) (possessionConf["height"]);
    possession.threshold = (int) (possessionConf["threshold"]);

    // create color window for the black robots
    FileNode obstacleConf = fs["obstacle"];
    obstacle.widthMin = 0; // not used
    obstacle.widthMax = 0; // not used
    obstacle.erodeSlider = (int) (obstacleConf["erodeSlider"]);
    obstacle.dilateSlider = (int) (obstacleConf["dilateSlider"]);
    obstacle.pixels = (int) (obstacleConf["pixels"]); // minimal required pixels detected by camera : countNonZero(inRangeFrame(rect))
    obstacle.xCenter = (int) (obstacleConf["xCenter"]); // correct for obstacle diameter in millimeters

    // create color window for the cyan label on robot
    FileNode cyanConf = fs["cyan"];
    cyan.widthMin = 0; // not used
    cyan.widthMax = 0; // not used
    cyan.erodeSlider = (int) (cyanConf["erodeSlider"]);
    cyan.dilateSlider = (int) (cyanConf["dilateSlider"]);
    cyan.pixels = (int) (cyanConf["pixels"]); // minimal required pixels detected by camera : countNonZero(inRangeFrame(rect))
    cyan.xCenter = (int) (cyanConf["xCenter"]); // correct for cyan diameter in millimeters

    // create color window for the magenta label on robot
    FileNode magentaConf = fs["magenta"];
    magenta.widthMin = 0; // not used
    magenta.widthMax = 0; // not used
    magenta.erodeSlider = (int) (magentaConf["erodeSlider"]);
    magenta.dilateSlider = (int) (magentaConf["dilateSlider"]);
    magenta.pixels = (int) (magentaConf["pixels"]); // minimal required pixels detected by camera : countNonZero(inRangeFrame(rect))
    magenta.xCenter = (int) (magentaConf["xCenter"]); // correct for magenta diameter in millimeters

    fs.release();

    lineStMutex.unlock();
    ballMutex.unlock();
    ballFarMutex.unlock();
    obstacleMutex.unlock();
    magentaMutex.unlock();
    cyanMutex.unlock();
}

void configurator::init(int floorWidth, int floorHeight, bool guiEnabled = false) {
    this->guiEnabled = guiEnabled;
    if (floorWidth % 2 == 0) {
        cerr << "multiCam: configurator Error: floor width should be an odd number but is: " << floorWidth << endl;
        ;
        exit(1);
    }
    if (floorHeight % 2 == 0) {
        cerr << "multiCam: configurator Error: floor height should be an odd number but is: " << floorHeight << endl;
        ;
        exit(1);
    }
    manual.x = (floorWidth - 1) / 2; // start in exact center
    manual.y = (floorHeight - 1) / 2;
    manual.rz = 90; // default shooter on y-axis

    if (guiEnabled) {
        showTrackbars(floorWidth, floorHeight);
    }

    updateLineErode(lineSt.erodeSlider);
    updateLineDilate(lineSt.dilateSlider);
    updateBallErode(ball.erodeSlider);
    updateBallFarErode(ball.erodeSlider);
    updateBallDilate(ball.dilateSlider);
    updateBallFarDilate(ball.dilateSlider);
    updateObstacleErode(obstacle.erodeSlider);
    updateObstacleDilate(obstacle.dilateSlider);
    updateCyanErode(cyan.erodeSlider);
    updateCyanDilate(cyan.dilateSlider);
    updateMagentaErode(magenta.erodeSlider);
    updateMagentaDilate(magenta.dilateSlider);
}

void configurator::showTrackbars(int floorWidth, int floorHeight) {

    // create the slide bars
    colorWindowLine = "line";
    namedWindow(colorWindowLine, cv::WINDOW_NORMAL);
    createTrackbar("l WidthMin", colorWindowLine, &lineSt.widthMin, 100);
    createTrackbar("l WidthMax", colorWindowLine, &lineSt.widthMax, 1000);
    createTrackbar("l Erode", colorWindowLine, &lineSt.erodeSlider, 20, &configurator::cbUpdateLineErodeTrackbar, this);
    createTrackbar("l Dilate", colorWindowLine, &lineSt.dilateSlider, 20, &configurator::cbUpdateLineDilateTrackbar,
            this);
    createTrackbar("l Distance", colorWindowLine, &lineSt.distance, 500);
    createTrackbar("l Distance Offset", colorWindowLine, &lineSt.distanceOffset, 200);
    createTrackbar("l Amount Maximal", colorWindowLine, &lineSt.linePointsNumberMaximal, 200);
    createTrackbar("l Amount Minimal", colorWindowLine, &lineSt.linePointsNumberMinimal, 200);

    colorWindowBall = "ball detection";
    namedWindow(colorWindowBall, cv::WINDOW_NORMAL);
    createTrackbar("b Erode", colorWindowBall, &ball.erodeSlider, 20, &configurator::cbUpdateBallErodeTrackbar, this);
    createTrackbar("b Dilate", colorWindowBall, &ball.dilateSlider, 20, &configurator::cbUpdateBallDilateTrackbar,
            this);
    createTrackbar("b Pixels", colorWindowBall, &ball.pixels, 300);
    createTrackbar("b Pixels Nearby", colorWindowBall, &ball.pixelsNearby, 5000);
    createTrackbar("b x Center", colorWindowBall, &ball.xCenter, 400);
    createTrackbar("b Azimuth", colorWindowBall, &ballAzimuthDeg, 100, &configurator::ballAzimuthTrackbarUpdate, this);

#ifdef NONO
    // balls and far away balls are combined on raspi
    colorWindowBallFar = "ball far detection";
    namedWindow(colorWindowBallFar, cv::WINDOW_NORMAL);
    createTrackbar("f Erode", colorWindowBallFar, &ballFar.erodeSlider, 20, &configurator::cbUpdateBallFarErodeTrackbar, this);
    createTrackbar("f Dilate", colorWindowBallFar, &ballFar.dilateSlider, 20, &configurator::cbUpdateBallFarDilateTrackbar,
            this);
    createTrackbar("f Pixels", colorWindowBallFar, &ballFar.pixels, 300);
    createTrackbar("f Pixels Max", colorWindowBallFar, &ballFar.pixelsMax, 1000);
    createTrackbar("f x Center", colorWindowBallFar, &ballFar.xCenter, 400);
    createTrackbar("f Azimuth", colorWindowBallFar, &ballFarAzimuthDeg, 100, &configurator::ballFarAzimuthTrackbarUpdate, this);
#endif

    colorWindowPossession = "possession";
    createTrackbar("p Width", colorWindowPossession, &possession.width, ROI_HEIGHT - 1);
    createTrackbar("p Height", colorWindowPossession, &possession.height, 100);
    createTrackbar("p Distance offset", colorWindowPossession, &possession.distanceOffset, 80);
    createTrackbar("p Threshold", colorWindowPossession, &possession.threshold, ROI_WIDTH - 1);


    colorWindowObstacle = "obstacle";
    namedWindow(colorWindowObstacle, cv::WINDOW_NORMAL);
    createTrackbar("o Erode", colorWindowObstacle, &obstacle.erodeSlider, 20,
            &configurator::cbUpdateObstacleErodeTrackbar, this);
    createTrackbar("o Dilate", colorWindowObstacle, &obstacle.dilateSlider, 20,
            &configurator::cbUpdateObstacleDilateTrackbar, this);
    createTrackbar("o Pixels", colorWindowObstacle, &obstacle.pixels, 4000);
    createTrackbar("o Distance offset", colorWindowObstacle, &obstacle.distanceOffset, 300);
    createTrackbar("o x center", colorWindowObstacle, &obstacle.xCenter, 600);
    createTrackbar("o Azimuth", colorWindowObstacle, &obstacleAzimuthDeg, 100,
            &configurator::obstacleAzimuthTrackbarUpdate, this);

    createTrackbar("o mask left far", colorWindowObstacle, &mask.leftFarAway, 400);
    createTrackbar("o mask left close", colorWindowObstacle, &mask.leftCloseby, 607);
    createTrackbar("o mask right close", colorWindowObstacle, &mask.rightCloseby, 607);
    createTrackbar("o mask right far", colorWindowObstacle, &mask.rightFarAway, 400);

    configWindow = "metrics";
    namedWindow(configWindow, cv::WINDOW_NORMAL);
    createTrackbar("x  Export-100/10", configWindow, &exportOffsetInt.x, 200); // center is 100, -100/10 to 100/10 = range -10 pixels to 10 pixels
    createTrackbar("y  Export-100/10", configWindow, &exportOffsetInt.y, 200); // center is 100, -100/10 to 100/10 = range -10 pixels to 10 pixels
    createTrackbar("rz Export-100/10", configWindow, &exportOffsetInt.rz, 200); // center is 100, -100/10 to 100/10 = range -10 degrees to 10 degrees
    createTrackbar("rz ExpBall-100/10", configWindow, &exportOffsetInt.rzBall, 200); // center is 100, -100/10 to 100/10 = range -10 degrees to 10 degrees
    createTrackbar("rz ExpObst-100/10", configWindow, &exportOffsetInt.rzObstacle, 200); // center is 100, -100/10 to 100/10 = range -10 degrees to 10 degrees

    createTrackbar("x  manual", configWindow, &manual.x, floorWidth - 1);
    createTrackbar("y  manual", configWindow, &manual.y, floorHeight - 1);
    createTrackbar("rz manual", configWindow, &manual.rz, 359);
    createTrackbar("xStep  solver", configWindow, &solver.xStep, 300);
    createTrackbar("yStep  solver", configWindow, &solver.yStep, 300);
    createTrackbar("rzStep solver", configWindow, &solver.rzStep, 200);
    createTrackbar("maxCnt solver", configWindow, &solver.maxCount, 400);
    // createTrackbar( "score manual", configWindow, &solver.score, 50);
    createTrackbar("blur pixels", configWindow, &blurPixels, 100);
    createTrackbar("pix to angle", configWindow, &pixelToAngle, 100);
    createTrackbar("x horizon", configWindow, &xHorizon, 550);
    createTrackbar("locLatencyOffset", configWindow, &locLatencyOffset, 500);
    createTrackbar("ballLatencyOffset", configWindow, &ballLatencyOffset, 500);
    createTrackbar("obstacleLatencyOffset", configWindow, &obstacleLatencyOffset, 500);
}

void configurator::update() {
}

colStruct configurator::getBall(size_t type) {
    if (type == ballType) {
        ballMutex.lock();
        colStruct tmp = ball;
        ballMutex.unlock();
        return tmp;
    } else if (type == ballFarType) {
        ballFarMutex.lock();
        colStruct tmp = ballFar;
        ballFarMutex.unlock();
        return tmp;
    } else if (type == cyanType) {
        cyanMutex.lock();
        colStruct tmp = cyan;
        cyanMutex.unlock();
        return tmp;
    } else if (type == magentaType) {
        magentaMutex.lock();
        colStruct tmp = magenta;
        magentaMutex.unlock();
        return tmp;
    } else if (type == obstacleType) {
        obstacleMutex.lock();
        colStruct tmp = obstacle;
        obstacleMutex.unlock();
        return tmp;
    } else {
        printf("ERROR     : type for configurator::getBall undefined %zu\n", type);
        exit( EXIT_FAILURE);
    }
}
