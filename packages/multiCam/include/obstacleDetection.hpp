 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
