// Copyright 2018-2022 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2014-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "multiCam.hpp"
#ifndef NOROS
#include "tracing.hpp"
#endif

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

    // TODO_APOX: quick hack to get the correct route for the raspi boards
    printf("INFO      : setup routes for 224.16.32.0 and 224.16.32.0 network\n");
    int tmp = system("sudo route add -net 224.16.16.0 netmask 255.255.255.0 dev enp0s31f6 >/tmp/multiCamRoute.log 2>&1");
    tmp += system("sudo ifconfig enp0s31f6 multicast >>/tmp/multiCamRoute.log 2>&1");

    // TODO_APOX: quick hack to get the correct route for diagnostics
    tmp += system("sudo route add -net 224.16.32.0 netmask 255.255.255.0 dev wlp2s0 >>/tmp/multiCamRoute.log 2>&1");
    tmp += system("sudo ifconfig wlp2s0 multicast >>/tmp/multiCamRoute.log 2>&1");

    tmp += system("sudo route add -net 224.16.32.0 netmask 255.255.255.0 dev wlp3s0 >>/tmp/multiCamRoute.log 2>&1");
    tmp += system("sudo ifconfig wlp3s0 multicast >>/tmp/multiCamRoute.log 2>&1");
    tmp += system("sudo route -n | grep 224 >>/tmp/multiCamRoute.log 2>&1");
    tmp += system("sudo route -n | grep -v virbr | grep -v docker");
    if( tmp != 0 ) {
        printf("INFO      : route return value sum %d\n", tmp);
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

    printf("INFO      : initializing dewarp ...\n");
    for (size_t cam = 0; cam < 4; cam++) {
        bool autoUpdate = (cam == 0);
        autoUpdate = false; // disable for now, checking if data repo is uptodate etc. seems a bit expensive to do each time
        dewarp[cam] = new Dewarper(robotId, cam, autoUpdate);
    }
    printf("INFO      : dewarp initialized\n");

    // setup ball distance estimator, shared by each ballDetection instance
    auto bde = new BallDistanceEstimator();

    for (size_t cam = 0; cam < 4; cam++) {
        // create for each camera a separate ball
        ballDet[cam] = new ballDetection(camAnaRecv, conf, dewarp[cam], bde, prep, ballType, cam);
        ballFarDet[cam] = new ballDetection(camAnaRecv, conf, dewarp[cam], bde, prep, ballFarType, cam);
        // start all ball and ballFar detectors, they will wait until data is received from the receiver
        ballDetThread[cam] = thread(&ballDetection::keepGoing, ballDet[cam]);
        ballFarDetThread[cam] = thread(&ballDetection::keepGoing, ballFarDet[cam]);

        // create for each camera a separate obstacle detector
        obstDet[cam] = new obstacleDetection(camAnaRecv, conf, dewarp[cam], prep, obstacleType, cam);
        // start all obstacle detectors, they will wait until data is received from the receiver
        // do not run obstacle detection on robot 1 (because of keeper frame first needs to be cut out)
        if (conf->getRobot() != 1) {
            obstDetThread[cam] = thread(&obstacleDetection::keepGoing, obstDet[cam]);
        }
    }

    // TODO: move cyanDet and megentaDet to above loop
    cyanDet = new obstacleDetection(camAnaRecv, conf, dewarp[0], prep, cyanType, 0); // TODO also for dewarp1 to dewarp3
    magentaDet = new obstacleDetection(camAnaRecv, conf, dewarp[0], prep, magentaType, 0); // TODO also for dewarp1 to dewarp3
    linePoint = new linePointDetection(camAnaRecv, conf, dewarp, prep);
    rFloor = new robotFloor(conf);
    detPos = new determinePosition(conf, linePoint, prep, rFloor);
    loc = new localization(conf, detPos, linePoint, prep);

    view = new viewer(ballDet, ballFarDet, camAnaRecv, camSysRecv, conf, cyanDet, detPos, linePoint, loc, magentaDet, obstDet, prep,
            rFloor, dewarp);
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
#ifndef NOROS
    TRACE_FUNCTION("");
    TRACE("multiCamLibrary::update start");
#endif
    conf->update();

#ifndef NOROS
    TRACE("multiCamLibrary::update waiting for new camera data ...");
#endif
    camAnaRecv->block(); // wait until new data is available, likely synchronized to camera 0 (front)
#ifndef NOROS
    TRACE("multiCamLibrary::update got new camera data ...");
#endif
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
                multSend->objectList(TYPE_BALLDETECTION);
                multSend->objectList(TYPE_BALLFARDETECTION);
                multSend->objectList(TYPE_CYANDETECTION);
                multSend->objectList(TYPE_MAGENTADETECTION);
                multSend->objectList(TYPE_OBSTACLES);
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
            capture.set(cv::CAP_PROP_POS_FRAMES, 0);
        } // r
        if (key == 's' || key == 65361 || key == '[') { // left arrow
            frameCurrent = frameCurrent - 10;
            if (frameCurrent < 0) {
                frameCurrent = 0;
            }
            capture.set(cv::CAP_PROP_POS_FRAMES, frameCurrent);
        }
        if (key == 'f' || key == 65362) { // up arrow
            frameCurrent = frameCurrent + 60;
            if (frameCurrent >= (capture.get(cv::CAP_PROP_FRAME_COUNT) - 1)) {
                frameCurrent = 0;
            }
            capture.set(cv::CAP_PROP_POS_FRAMES, frameCurrent);
        }
        if (key == 'd' || key == 65363 || key == ']') { // right arrow
            frameCurrent = frameCurrent + 10;
            if (frameCurrent >= (capture.get(cv::CAP_PROP_FRAME_COUNT) - 1)) {
                frameCurrent = 0;
            }
            capture.set(cv::CAP_PROP_POS_FRAMES, frameCurrent);
        }
        if (key == 'a' || key == 65364) { // down arrow
            frameCurrent = frameCurrent - 60;
            capture.set(cv::CAP_PROP_POS_FRAMES, frameCurrent);
            if (frameCurrent < 0) {
                frameCurrent = 0;
            }
        }
        if (key == 'x') {
            frameCurrent = frameCurrent + 1;
            if (frameCurrent >= (capture.get(cv::CAP_PROP_FRAME_COUNT) - 1)) {
                frameCurrent = 0;
            }
            capture.set(cv::CAP_PROP_POS_FRAMES, frameCurrent);
        }
        if (key == 'z') {
            frameCurrent = frameCurrent - 1;
            capture.set(cv::CAP_PROP_POS_FRAMES, frameCurrent);
            if (frameCurrent < 0) {
                frameCurrent = 0;
            }
        }

        if (view->getPause()) {
            capture.set(cv::CAP_PROP_POS_FRAMES, frameCurrent);
        }

        if (frameCurrent >= (capture.get(cv::CAP_PROP_FRAME_COUNT) - 1)) {
            frameCurrent = 0;
            capture.set(cv::CAP_PROP_POS_FRAMES, 0);
        }
    }

#ifndef NOROS
    TRACE("multiCamLibrary::update end");
    WRITE_TRACE;
#endif
    return true;
}
