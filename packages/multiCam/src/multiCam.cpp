 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2014-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "multiCam.hpp"

#include <cerrno>
#include <fcntl.h>    /* For O_RDWR */
#include <pwd.h>
#include <unistd.h>

using namespace std;
using namespace cv;

multiCamLibrary::multiCamLibrary(int robotIdArg, bool guiEnabled) {
    this->guiEnabled = guiEnabled;

    printf("INFO      : openCV version %d.%d.%d\n", CV_MAJOR_VERSION, CV_MINOR_VERSION, CV_VERSION_REVISION);

    frameCurrent = 0;
    useCamera = false;
    previousSendTime = 0.0;

    robotId = 0;
    if (robotIdArg != 0) {
        robotId = robotIdArg;
    } else {
        // if robotId not provided as command line argument, use the TURTLE5K_ROBOTNUMBER environment variable
        if (getenv("TURTLE5K_ROBOTNUMBER") != NULL) {
            robotId = atoi(getenv("TURTLE5K_ROBOTNUMBER"));
            printf("INFO      : using environment variable TURTLE5K_ROBOTNUMBER to set robot id to %d\n", robotId);
        }
    }

    if (robotId < 0 || robotId > 8) {
        printf("ERROR     : robotId %d is out of range 1 to 8\n", robotId);
        exit( EXIT_FAILURE);
    }

    conf = new configurator(robotId);
    prep = new preprocessor(conf);
    camAnaRecv = new cameraReceive(conf);
    camSysRecv = new camSysReceive();
    // update and acquire the md5sum of all relevant files
    camSysRecv->getLocalMd5sum();
    // enable the md5sum check for multiCam
    camSysRecv->setMd5sumCheck(true);
    // use volatile and fast (no disk access) /dev/shm to store the received jpg files
    camSysRecv->setImageGrabPath("/dev/shm");

    // setup the dewarp
    std::thread dewarpThread[4];

    printf("INFO      : dewarp calculate start\n");
    for (size_t cam = 0; cam < 4; cam++) {
        // set the config file for the dewarp
        // TODO: move file name to yaml
        struct passwd *pw = getpwuid(getuid());
        std::string configFile("");
        configFile.append(pw->pw_dir);
        switch (robotId) {
        case 1:
            configFile.append(
                    "/falcons/data/internal/vision/multiCam/calibration/20190223_r1_assyG_cam" + std::to_string(cam)
                            + ".bin");
            break;
        case 2:
            configFile.append(
                    "/falcons/data/internal/vision/multiCam/calibration/20190219_r2_assyI_cam" + std::to_string(cam)
                            + ".bin");
            break;
        case 3:
            configFile.append(
                    "/falcons/data/internal/vision/multiCam/calibration/20190219_r3_assyD_cam" + std::to_string(cam)
                            + ".bin");
            break;
        case 4:
            configFile.append(
                    "/falcons/data/internal/vision/multiCam/calibration/20180927_r4_assyF_cam" + std::to_string(cam)
                            + ".bin");
            break;
        case 5:
            configFile.append(
                    "/falcons/data/internal/vision/multiCam/calibration/20190117_r5_assyE_cam" + std::to_string(cam)
                            + ".bin");
            break;
        case 6:
            configFile.append(
                    "/falcons/data/internal/vision/multiCam/calibration/20190611_r6_assyH_cam" + std::to_string(cam)
                            + ".bin");
            break;
        case 7:
            configFile.append(
                    "/falcons/data/internal/vision/multiCam/calibration/20190402_r7_cam" + std::to_string(cam)
                            + ".bin");
            break;
        case 8:
            // WARNING: test only, re-using calibration data or robot 7
            configFile.append(
                    "/falcons/data/internal/vision/multiCam/calibration/20190402_r7_cam" + std::to_string(cam)
                            + ".bin");
            break;
        }

        // setup the dewarp
        dewarp[cam] = new deWarper();
        dewarp[cam]->readBIN(configFile);
    }

    printf("INFO      : dewarp initialized\n");
    // dewarp[0]->verify();

    for (size_t cam = 0; cam < 4; cam++) {
        // create for each camera a separate ball and ballFar detector
        ballDet[cam] = new ballDetection(camAnaRecv, conf, dewarp[cam], prep, ballType, cam);
        ballFarDet[cam] = new ballDetection(camAnaRecv, conf, dewarp[cam], prep, ballFarType, cam);
        // start all ball and ballFar detectors, they will wait until data is received from the receiver
        ballDetThread[cam] = thread(&ballDetection::keepGoing, ballDet[cam]);
        ballFarDetThread[cam] = thread(&ballDetection::keepGoing, ballFarDet[cam]);

        // create for each camera a separate obstacle detector
        obstDet[cam] = new ballDetection(camAnaRecv, conf, dewarp[cam], prep, obstacleType, cam);
        // start all obstacle detectors, they will wait until data is received from the receiver
        // do not run obstacle detection on robot 1 (because of keeper frame first needs to be cut out)
        if (conf->getRobot() != 1) {
            obstDetThread[cam] = thread(&ballDetection::keepGoing, obstDet[cam]);
        }
    }

    // TODO: move cyanDet and megentaDet to above loop
    cyanDet = new ballDetection(camAnaRecv, conf, dewarp[0], prep, cyanType, 0); // TODO also for dewarp1 to dewarp3
    magentaDet = new ballDetection(camAnaRecv, conf, dewarp[0], prep, magentaType, 0); // TODO also for dewarp1 to dewarp3
    linePoint = new linePointDetection(camAnaRecv, conf, dewarp, prep);
    rFloor = new robotFloor(conf);
    detPos = new determinePosition(conf, linePoint, prep, rFloor);
    loc = new localization(conf, detPos, linePoint, prep);

    view = new viewer(ballDet, ballFarDet, camAnaRecv, camSysRecv, conf, cyanDet, detPos, linePoint, loc, magentaDet, obstDet, prep,
            rFloor);
    // obsolete: diag = new cFrameDiagnostics(conf, rFloor, view);
    // TODO: multSend all 4 ball detectors
    multSend = new multicastSend(ballDet, ballFarDet, camAnaRecv, camSysRecv, conf, cyanDet, detPos, linePoint, loc, magentaDet,
            obstDet, prep, rFloor);

    conf->init(rFloor->getWidth(), rFloor->getHeight(), guiEnabled);

    // start thread that collects analyze data from multiple camera's
    camAnaRecvThread = thread(&cameraReceive::receive, camAnaRecv);

    // start thread that collects system data from multiple camera's
    camSysRecvThread = thread(&camSysReceive::receive, camSysRecv);
}

void multiCamLibrary::attach(observer *obs) {
    detPos->attach(obs);
    multSend->attach(obs);
    for (size_t cam = 0; cam < 4; cam++) {
        ballDet[cam]->attach(obs);
        ballFarDet[cam]->attach(obs); // TODO: investigate if this will work with ros / rtdb
        obstDet[cam]->attach(obs);
    }
}

bool multiCamLibrary::update() {
    conf->update();

    camAnaRecv->block(); // wait until new data is available, likely synchronized to camera 0 (front)
    prep->update();

    if (!loc->getBusy()) { // only start new localization thread when previous thread was finished
        if (locThread.joinable()) { // this should be almost always the case
            locThread.join(); // be sure to join the previous tread before we start a new one
            double uptime = prep->getUptime();
            double multicastSendPeriod = 1.0 / conf->getMulticast().frequency;
            if (uptime > (previousSendTime + multicastSendPeriod)) {
                multSend->stats(); // used to invalidate other packets, send as first packet
                multSend->goodEnoughLoc(); // goodEnough is used by the others, except stats(), send as second packet
                multSend->floorLinePoints();
                multSend->locList();
                multSend->ballList(TYPE_BALLDETECTION);
                multSend->ballList(TYPE_BALLFARDETECTION);
                multSend->ballList(TYPE_CYANDETECTION);
                multSend->ballList(TYPE_MAGENTADETECTION);
                multSend->ballList(TYPE_OBSTACLES);
                // increase previousSendTime for the next send
                while (uptime > (previousSendTime + multicastSendPeriod)) {
                    previousSendTime += multicastSendPeriod;
                }
            }
        }
        loc->setBusy(); // semaphore
        locThread = thread(&localization::update, loc);
    }
    if (!view->getBusy()) { // only start new viewer thread when previous thread has finished
        if (viewerThread.joinable()) { // this should be almost always the case
            viewerThread.join(); // be sure to join the previous tread before we start a new one
        }
        view->setBusy(); // semaphore
        viewerThread = thread(&viewer::update, view);
    }

    rFloor->update(); // used when swap from normal mode to calibration mode

    int key = view->getKey();
    if (key == 27 || key == 'q') // escape or q
            {
        return false;
    }
    if (!useCamera) {
        // following keys are only available when video is read from file instead of camera (which is when frameCurrent >= 0)
        if (key == 'r') {
            capture.set(CV_CAP_PROP_POS_FRAMES, 0);
        } // r
        if (key == 's' || key == 65361 || key == '[') { // left arrow
            frameCurrent = frameCurrent - 10;
            if (frameCurrent < 0) {
                frameCurrent = 0;
            }
            capture.set(CV_CAP_PROP_POS_FRAMES, frameCurrent);
        }
        if (key == 'f' || key == 65362) { // up arrow
            frameCurrent = frameCurrent + 60;
            if (frameCurrent >= (capture.get(CV_CAP_PROP_FRAME_COUNT) - 1)) {
                frameCurrent = 0;
            }
            capture.set(CV_CAP_PROP_POS_FRAMES, frameCurrent);
        }
        if (key == 'd' || key == 65363 || key == ']') { // right arrow
            frameCurrent = frameCurrent + 10;
            if (frameCurrent >= (capture.get(CV_CAP_PROP_FRAME_COUNT) - 1)) {
                frameCurrent = 0;
            }
            capture.set(CV_CAP_PROP_POS_FRAMES, frameCurrent);
        }
        if (key == 'a' || key == 65364) { // down arrow
            frameCurrent = frameCurrent - 60;
            capture.set(CV_CAP_PROP_POS_FRAMES, frameCurrent);
            if (frameCurrent < 0) {
                frameCurrent = 0;
            }
        }
        if (key == 'x') {
            frameCurrent = frameCurrent + 1;
            if (frameCurrent >= (capture.get(CV_CAP_PROP_FRAME_COUNT) - 1)) {
                frameCurrent = 0;
            }
            capture.set(CV_CAP_PROP_POS_FRAMES, frameCurrent);
        }
        if (key == 'z') {
            frameCurrent = frameCurrent - 1;
            capture.set(CV_CAP_PROP_POS_FRAMES, frameCurrent);
            if (frameCurrent < 0) {
                frameCurrent = 0;
            }
        }

        if (view->getPause()) {
            capture.set(CV_CAP_PROP_POS_FRAMES, frameCurrent);
        }

        if (frameCurrent >= (capture.get(CV_CAP_PROP_FRAME_COUNT) - 1)) {
            frameCurrent = 0;
            capture.set(CV_CAP_PROP_POS_FRAMES, 0);
        }
    }

	return true;
}
