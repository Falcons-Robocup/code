 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Jan Feitsma, december 2019

#include "obstacleDetection.hpp"

#include <math.h>

using namespace std;
using namespace cv;

obstacleDetection::obstacleDetection(cameraReceive *camRecv, configurator *conf, Dewarper *dewarp, preprocessor *prep,
        size_t type, size_t camIndex) {
    this->camRecv = camRecv;
    this->conf = conf;
    this->dewarp = dewarp;
    this->prep = prep;
    this->type = type;
    this->camIndex = camIndex;

    height = conf->getCameraHeight();
    width = conf->getCameraWidth();

    exportMutex.lock();
    // initialize frames so even without running the update method the frames can be used by the viewer
    erodeFrame = Mat::zeros(height, width, CV_8UC1);
    dilateFrame = Mat::zeros(height, width, CV_8UC1);
    inRangeFrame = Mat::zeros(height, width, CV_8UC1);

    // make the result list empty so the first request does not get bogus data
    positions.clear();
    positionsRemoteViewerExportMutex.unlock();
    obstaclePointListExportMutex.unlock();
    exportMutex.unlock();
    busy = false;
    printCount = 0;
    exportMutex.lock();
    for (size_t ii = 0; ii < height; ii++) { // create vector for the closest obstacle pixels
        closestPixels.push_back(-1);
        closestGroups.push_back(-1);
    }
    exportMutex.unlock();
}

void obstacleDetection::keepGoing() {
    while (1) {
        update();
    }
}

typedef enum closestState {
    CLOSEST_INIT = 0, CLOSEST_NOTHING, CLOSEST_FOUND, CLOSEST_PENDING, CLOSEST_STORE,
} closestStateT;

// try to find the closest line for obstacles
// first get all closest obstacle points
// then group them together
// use these groups to split the dilated obstacle image, so the borders are separated
// and even robot close to each other (but at a different distance)
// NOTE: the camera is rotated, so the nearby axis the y axis (x = 0)
void obstacleDetection::findClosestLineGroups() {
    // run through all lines
    for (size_t yy = 0; yy < height; yy++) {
        // get the closest pixel of this particular line
        // Note: the lock mutex is already set by the caller of this function
        closestPixels[yy] = -1; // default closest obstacle pixel not found
        for (size_t xx = 0; xx < width; xx++) {
            uchar pixelVal = inRangeFrame.at<uchar>(yy, xx);
            if (pixelVal != 0) {
                // found the obstacle pixel, store and goto next line
                closestPixels[yy] = xx;
                xx = width; // found it, go to next line
            }
        }
    }

    // now group the pixels, each group represents one obstacle

    // initialize the closestGroup vector, meaning the closest pixels are not related to any group
    for (size_t yy = 0; yy < height; yy++) {
        closestGroups[yy] = -1; // not assigned to any group
    }

    ssize_t groupIndex = 0;

    for (ssize_t yy0 = 0; yy0 < height - 1; yy0++) {
        ssize_t xx0 = closestPixels[yy0];
        if (xx0 >= 0) {
            if (closestGroups[yy0] < 0) {
                // this pixel was not yet related to a group, assign to new group
                closestGroups[yy0] = groupIndex;
                groupIndex++; // prepare for next group
            }

            // determine distance to all of the remaining pixels
            for (ssize_t yy1 = yy0 + 1; yy1 < height; yy1++) {
                ssize_t xx1 = closestPixels[yy1];
                if (xx1 >= 0) {
                    double delta = sqrt(pow(xx0 - xx1, 2) + pow(yy0 - yy1, 2));
                    double threshold = (xx1 - 470.0) / -16.0;
                    if (threshold > 15.0) {
                        threshold = 15.0;
                    }
                    if (threshold < 3.0) {
                        threshold = 3.0;
                    }
                    if (delta < threshold) {
                        // this obstacle pixels is close to the yy0 obstacle pixels, assign to same group
                        closestGroups[yy1] = closestGroups[yy0];
                    }
                }
            }
        }
    }
    // now we know for each closest pixel if it belongs to a group and if so to which group
}

// Search for obstacles in the related camera
// Pixels are filtered by the Erode, Dilate and the minimum size
// The dewarp function is used to convert the obstacle (bottom) camera pixels to x,y offset in mm
// The x,y position is relative to the robot (and camera on that robot)
// The camera is 90 degrees rotated, so the x mainly represents the distance between the
// robot and the obstacle, while the y mainly represents the angle
// An y of 0 means the obstacle is straight for the camera, and negative y means the obstacle
// is at the left side of the camera and a positive y, the obstacle is at the right
// side of the robot
// The x,y Cartesian coordinate converted to Polar coordinates, to make them
// easier to use as relative to the Robot
// Note: The radius is measured from the closest yellow pixel, but because of the camera position, this
// is roughly the same as the center of the obstacle.
void obstacleDetection::update() {
// get the pixels of the camera
    vector<linePointSt> cartesian;

// wait for points, delivered by the receiver
// Note: one point is actually a line (xBegin,y) and (xEnd,y)
    if (type == ballType) {
        printf("ERROR     : obstacleDetection is not intended for ball detection\n");
        exit(EXIT_FAILURE);
    } else if (type == ballFarType) {
        printf("ERROR     : obstacleDetection is not intended for FAR ball detection\n");
        exit(EXIT_FAILURE);
    } else if (type == obstacleType) {
        cartesian = camRecv->getObstaclePointsWait(camIndex);
    } else {
        printf("ERROR     : type %zu unknown\n", type);
        exit( EXIT_FAILURE);
    }

    obstaclePointListExportMutex.lock();
    obstaclePointList = cartesian;
    obstaclePointListExportMutex.unlock();

// construct "image" from received points
    exportMutex.lock();
    inRangeFrame = Mat::zeros(height, width, CV_8UC1);
    for (size_t ii = 0; ii < cartesian.size(); ii++) {
        int xBegin = cartesian[ii].xBegin;
        if (xBegin < conf->getBall(type).distanceOffset) {
            xBegin = conf->getBall(type).distanceOffset;
        }
        int xEnd = cartesian[ii].xEnd;
        int y = cartesian[ii].yBegin;
        // TODO: use yEnd to create a rectangle instead of line
        // TODO: set line width back to 1 if all lines are scanned in grabber (now only half the lines used)
        if (xEnd >= xBegin) {
            // xEnd can be lower the xBegin because of using the distance offset (mainly required for obstacles because of the camera cover)
            line(inRangeFrame, Point(xBegin, y), Point(xEnd, y), 255, 2, 0);
        }
    }

// mask camera cover to prevent false obstacles (and fake balls)
// create two triangles that block the fake obstacles
// left triangle
    int blockColor = 0;
    Point blockTriangle[1][3];
    blockTriangle[0][0] = Point(0, 0);
    blockTriangle[0][1] = Point(0, conf->getMask().leftCloseby);
    blockTriangle[0][2] = Point(conf->getMask().leftFarAway, 0);

    const Point* pptLeft[1] = { blockTriangle[0] };
    int npt[] = { 3 };
    fillPoly(inRangeFrame, pptLeft, npt, 1, blockColor, 0);

// right triangle
    blockTriangle[0][0] = Point(0, ROI_HEIGHT - 1);
    blockTriangle[0][1] = Point(0, conf->getMask().rightCloseby);
    blockTriangle[0][2] = Point(conf->getMask().rightFarAway, ROI_HEIGHT - 1);

    const Point* pptRight[1] = { blockTriangle[0] };
    fillPoly(inRangeFrame, pptRight, npt, 1, blockColor, 0);

    // determine obstacle groups from the closest by obstacle pixels
    if (type == obstacleType) {
        findClosestLineGroups();
    }

// typically there are points of 1 pixel wide, likely caused by the demosaicing in the FPGA
    erode(inRangeFrame, erodeFrame, conf->getBall(type).erode);            // reduce
// now cluster together what is left
    dilate(erodeFrame, dilateFrame, conf->getBall(type).dilate);            // expand

    // use the closest by obstacle groups to separate the dilated frame, so the contours will not stretch
    // over multiple obstacles (e.g. black border)
    ssize_t groupIndexPrev = closestGroups[0];
    for (ssize_t yy = 1; yy < height; yy++) {
        ssize_t groupIndex = closestGroups[yy];
        // add a blocking line when the group is changing or no obstacle pixel found on line
        if ((groupIndexPrev != groupIndex) || (groupIndex == -1)) {
            groupIndexPrev = groupIndex;
            // make the blocking line wider if the dilate is wider to remove dilated pixels on the other side of the blocking line
            int lineWidth = conf->getBall(type).dilateSlider;
            line(dilateFrame, Point(0, yy), Point(width - 1, yy), 0, lineWidth);
        }
    }

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

// determine contours that will encapsulate the clusters
    Mat tmp;
    dilateFrame.copyTo(tmp);            // copy dilateFrame otherwise it will be modified by findContours
    findContours(tmp, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, Point(0, 0));

    positions.clear();            // start with empty list
    for (size_t ii = 0; ii < contours.size(); ii++) {
        // make a bounding rectangle around each contour to calculate the distance, center and amount of obstacle pixels
        Rect rect = boundingRect(contours.at(ii));

        // determine the pixel closest by to the obstacle (radius)
        int xClosestBy = rect.x;

        // determine the center of the obstacle (azimuth)
        int yCenter = rect.y + rect.height / 2;

        // determine the amount of obstacle pixels inside the bounding rectangle
        int pixels = countNonZero(inRangeFrame(rect));

        // convert from Cartesian to Polar
        int16_t xField = 0;
        int16_t yField = 0;
        obstacleSt polar;

        // diagnostics
        bool attemptedDewarp = false;
        bool usedDewarp = false;
        bool enoughPixels = pixels > conf->getBall(type).pixels;

        // the height of an obstacle (robot) is larger then the width
        // however there are cases when 2 robots are next to each other
        // so only reject when obstacle is really low
        // this reject the black border lines
        if (rect.height > (2 * rect.width)) { // note: camera is rotated
            enoughPixels = 0;
        }

        // close by obstacles should have a significant size
        // TODO: make use of (configurable) curve instead of fixed values
        if (xClosestBy < 25) {
            if (pixels < 6000) { // verify with falcons/data/internal/vision/multiCam/r2/cam0_20190705_084029.jpg
                enoughPixels = 0;
            } else {
                // printf("INFO   : cam %zu obstacle x closest %d pixels %d\n", camIndex, xClosestBy, pixels);
            }
        } else if (xClosestBy < 50) {
            if (pixels < 4000) {
                enoughPixels = 0;
            } else {
                // printf("INFO   : cam %zu obstacle x closest %d pixels %d\n", camIndex, xClosestBy, pixels);
            }
        } else if (xClosestBy < 100) {
            if (pixels < 3000) {
                enoughPixels = 0;
            }
        } else if (xClosestBy < 150) {
            if (pixels < 2000) {
                enoughPixels = 0;
            }
        } else if (xClosestBy < 200) {
            if (pixels < 1000) {
                enoughPixels = 0;
            }
        }
        // for farther away the configuration slider is used

        // check if there are enough pixels to qualify for a obstacle
        if (enoughPixels) {
            bool valid = false;
            polar.size = pixels;
            polar.xClosestBy = xClosestBy;
            polar.yCenter = yCenter;
            polar.rect = rect;

            // convert from camera dimensions (pixels) to field dimensions (meters)

            // obstacles are by definition ALWAYS on the floor (z == 0)
            // so we use dewarp (also used for line points) to determine the relative location of the ball
            float azimuthDewarp = 0.0;
            float elevationDewarp = 0.0;
            float radiusDewarpMm = FLT_MAX;
            if ((xClosestBy >= 0) && (xClosestBy < (ROI_WIDTH / 2)) && (yCenter < ROI_HEIGHT)) {
                // TODO: move the dewarp to he camRecv
                // the correct dewarp already selected on the multiCam level
                bool ok = dewarp->transformFloor(xClosestBy, yCenter, yField, xField);                // xField and yField result in mm

                if (ok) {
                    // dewarp is calibrated for the pixel input at center of the line, while the ball and obstacle detection
                    // provide the first dilate pixel, compensate to provide the center of the ball or obstacle
                    double xFieldRadiusCorrected = xField + conf->getBall(type).xCenter;                        // in mm
                    radiusDewarpMm = sqrt(xFieldRadiusCorrected * xFieldRadiusCorrected + yField * yField);        // in mm

                    // the dewarp is calibrated for the lines, the xField is closest by
                    // TODO: checkout why angle needs to be set negative over here
                    // would expect the x and y would have the correct polarity and atan2 should then also be correct
                    azimuthDewarp = -atan2(yField, xFieldRadiusCorrected);
                    // polar.azimuth = -polar.azimuth - CV_PI/2.0; // new calibration uses inverted angles - 90 degrees
                    // Note: the obstacleDetection is relative to it's own camera, so adding the 90 degrees per camera's will be performed
                    // on the higher level
                    elevationDewarp = atan2(-CAMERA_HEIGHT, 1e-3 * radiusDewarpMm);    // yikes, careful with MM scaling ...
                    valid = true;
                } // if xField
                attemptedDewarp = true;
            } // if xClosestBy

            // obstacle on the floor
            polar.azimuth = azimuthDewarp;
            polar.elevation = elevationDewarp;
            polar.radius = radiusDewarpMm;
            usedDewarp = true;

            if (valid) {

                char typeStr[32];
                sprintf(typeStr, "obstacle");

// azimuth range: horizontal view angle is around -60 to 60 = 120 degrees (of which 90 is used)
                // however the camera cover of robot 4 generates obstacles with a azimuth of 77 degrees
#define AZIMUTH 90.0
                // elevation range: down -80 degrees to 80 degrees = 160 degree, robot 3 is -81 degrees, add some more headroor
#define ELEVATION 85.0

                if ((polar.azimuth < -(M_PI * AZIMUTH / 180.0)) || (polar.azimuth > (M_PI * AZIMUTH / 180.0))) {
                    printf(
                            "ERROR     : cam %zu %s azimuth %5.1f out of range, elevation %5.1f radius %6.0f size %6d x closest by %4d y center %4d y width %4d\n",
                            camIndex, typeStr, 180.0 * polar.azimuth / M_PI, 180.0 * polar.elevation / M_PI,
                            polar.radius, polar.size, polar.xClosestBy, polar.yCenter, polar.rect.height);
                } else if ((polar.elevation < (-M_PI * ELEVATION / 180.0))
                        || (polar.elevation > ( M_PI * ELEVATION / 180.0))) {
                    printf(
                            "ERROR     : cam %zu %s elevation %5.1f out of range, azimuth %5.1f radius %6.0f size %6d x closest by %4d y center %4d y width %4d\n",
                            camIndex, typeStr, 180.0 * polar.elevation / M_PI, 180.0 * polar.azimuth / M_PI,
                            polar.radius, polar.size, polar.xClosestBy, polar.yCenter, polar.rect.height);
                } else if ((polar.xClosestBy < 0) || (polar.xClosestBy >= ROI_WIDTH)) {
                    printf(
                            "ERROR     : cam %zu %s x out of range azimuth %5.1f elevation %5.1f radius %6.0f size %6d x closest by %4d y center %4d y width %4d\n",
                            camIndex, typeStr, 180.0 * polar.azimuth / M_PI, 180.0 * polar.elevation / M_PI,
                            polar.radius, polar.size, polar.xClosestBy, polar.yCenter, polar.rect.height);
                } else if ((polar.yCenter < 0) || (polar.yCenter >= ROI_HEIGHT)) {
                    printf(
                            "ERROR     : cam %zu %s y out of range azimuth %5.1f elevation %5.1f radius %6.0f size %6d x closest by %4d y center %4d y width %4d\n",
                            camIndex, typeStr, 180.0 * polar.azimuth / M_PI, 180.0 * polar.elevation / M_PI,
                            polar.radius, polar.size, polar.xClosestBy, polar.yCenter, polar.rect.height);
                } else if ((polar.rect.height < 0) || (polar.rect.height > ROI_HEIGHT)) {
                    printf(
                            "ERROR     : cam %zu %s rect out of range azimuth %5.1f elevation %5.1f radius %6.0f size %6d x closest by %4d y center %4d y width %4d\n",
                            camIndex, typeStr, 180.0 * polar.azimuth / M_PI, 180.0 * polar.elevation / M_PI,
                            polar.radius, polar.size, polar.xClosestBy, polar.yCenter, polar.rect.height);
                } else if ((polar.size < 0) || (polar.size > (ROI_WIDTH * ROI_HEIGHT))) {
                    printf(
                            "ERROR     : cam %zu %s size out of range azimuth %5.1f elevation %5.1f radius %6.0f size %6d x closest by %4d y center %4d y width %4d\n",
                            camIndex, typeStr, 180.0 * polar.azimuth / M_PI, 180.0 * polar.elevation / M_PI,
                            polar.radius, polar.size, polar.xClosestBy, polar.yCenter, polar.rect.height);
                } else if ((polar.radius < 0) || (polar.radius > 20000)) {
                    printf(
                            "ERROR     : cam %zu %s radius out of range azimuth %5.1f elevation %5.1f radius %6.0f size %6d x closest by %4d y center %4d y width %4d\n",
                            camIndex, typeStr, 180.0 * polar.azimuth / M_PI, 180.0 * polar.elevation / M_PI,
                            polar.radius, polar.size, polar.xClosestBy, polar.yCenter, polar.rect.height);
                } else {
                    // each camera should be able to detect 90 degrees (=> -45 degrees to 45 degrees)
                    // because of tolerances a ball on the 45 degree line might be missed
                    // with a range of 46 degrees a ball might be reported by 2 camera's
                    // TODO: add function that removes double balls (or let world model deal with it)
                    float azimuth = conf->getBall(type).azimuth;
                    if ((polar.azimuth >= -azimuth) && (polar.azimuth <= azimuth)) {
                        positions.push_back(polar);
#ifdef NONO
                    } else {
                        printf("WARNING : cam %zu obstacle angle of %.0f degrees should be covered by other camera\n",
                                    camIndex, 180.0 * polar.azimuth / M_PI);
#endif
                    } // if polar.
                } // valid
            } // if enoughPixels
        } // for contours.size()

        // containment for compile warnings about not used diagnostics variables
        (void) attemptedDewarp;
        (void) usedDewarp;

    }

// sort on size
    sort(positions.begin(), positions.end());
    positionsRemoteViewerExportMutex.lock();
    positionsRemoteViewer = positions;
    positionsRemoteViewerExportMutex.unlock();
    exportMutex.unlock();

    if (positions.size() > 0) {
// Notify new ball position
        notifyNewPos();
    }
    busy = false;

}

vector<obstacleSt> obstacleDetection::getPositions() {
    exportMutex.lock();
    std::vector<obstacleSt> retVal = positions;
    exportMutex.unlock();
    return retVal;
}

vector<ssize_t> obstacleDetection::getClosestPixels() {
    exportMutex.lock();
    std::vector<ssize_t> retVal = closestPixels;
    exportMutex.unlock();
    return retVal;
}

vector<ssize_t> obstacleDetection::getClosestGroups() {
    exportMutex.lock();
    std::vector<ssize_t> retVal = closestGroups;
    exportMutex.unlock();
    return retVal;
}

vector<obstacleSt> obstacleDetection::getAndClearPositionsRemoteViewer() {
    positionsRemoteViewerExportMutex.lock();
    std::vector<obstacleSt> retVal = positionsRemoteViewer;
    positionsRemoteViewer.clear();
    positionsRemoteViewerExportMutex.unlock();
    return retVal;
}

vector<obstacleSt> obstacleDetection::getPositionsExport() {
    vector<obstacleSt> retVal = getPositions();
    for (size_t ii = 0; ii < retVal.size(); ii++) {
        double tmpAzimuth = retVal[ii].azimuth; // TODO: checkout if this is still needed, and if, convert to radians + conf->getExportOffset().rzBall;
// normalize to prevent issues when running of range for remote viewer export
        retVal[ii].azimuth = fmod(2 * CV_PI + tmpAzimuth, 2 * CV_PI);
        retVal[ii].elevation = retVal[ii].elevation; // no fmod, kthxbye
    }
    return retVal;
}

cv::Mat obstacleDetection::getDilateFrame() {
    exportMutex.lock();
    cv::Mat retVal = dilateFrame.clone();
    exportMutex.unlock();
    return retVal;
}

cv::Mat obstacleDetection::getErodeFrame() {
    exportMutex.lock();
    cv::Mat retVal = erodeFrame.clone();
    exportMutex.unlock();
    return retVal;
}

cv::Mat obstacleDetection::getInRangeFrame() {
    exportMutex.lock();
    cv::Mat retVal = inRangeFrame.clone();
    exportMutex.unlock();
    return retVal;
}

size_t obstacleDetection::getAmount() {
    size_t amount = 0;
    exportMutex.lock();
    for (size_t ii = 0; ii < positions.size(); ii++) {
        if (positions[ii].size >= conf->getBall(type).pixels) {
            amount++;
        }
    }
    exportMutex.unlock();
    return amount;
}

void obstacleDetection::notifyNewPos() {
    vector<obstacleSt> exportPos = getPositionsExport();

#ifndef NOROS
    if (type != obstacleType) {
        std::vector<ballPositionType> balls;

        for (uint ii = 0; ii < exportPos.size(); ii++) {
            double size = exportPos[ii].size;
            if (size > conf->getBall(type).pixels) {
                // Note: the world model needs to discard balls that are outside the field e.g. yellow something around the field
                // this is depending on the final position, which is determined by the world model instead of the localization on this robot
                double angleRad = exportPos[ii].azimuth;// counter clockwise angle between this robot shooter and the obstacle

                // cam 1 is left of cam 0 = counterclockwise
                angleRad += camIndex * CV_PI / 2.0;            // camIndex * 90 degrees, every camera is 90 degrees rotated

                // from center of this robot to closest edge of ball, there is no need to add a ballRadius to radius\ (ball edge) because in field view the radius is very close to the center
                // exportPos[ii].radius is in mm, while Ros expects meters
                double radius = 0.001f * exportPos[ii].radius;
                double score = size / 100.0f;
                if (score > 1.0) {
                    score = 1.0;
                }
                // cout << "ball index " << ii << " size " << size << " score " << score << " << " angle radians " << angleRad << " radius " << radius << endl;

                ballPositionType ball;

                ball.setAngle(angleRad);
                ball.setRadius(radius);
                ball.setConfidence(score);
                ball.setElevation(exportPos[ii].elevation);
                ball.setBallType(type);

                balls.push_back(ball);
            }
        }

        for (vector<observer*>::const_iterator iter = vecObservers.begin(); iter != vecObservers.end(); ++iter) {
            if (*iter != NULL) {
                (*iter)->update_own_ball_position(balls, conf->getBallLatencyOffset());
            }
        }
    } else {
        std::vector<obstaclePositionType> obstacles;

        for (uint ii = 0; ii < exportPos.size(); ii++) {
            double size = exportPos[ii].size;
            if (size > conf->getBall(type).pixels) { // type is in this case obstacle
                // Note: the world model needs to discard balls that are outside the field e.g. yellow something around the field
                // this is depending on the final position, which is determined by the world model instead of the localization on this robot
                double angleRad = exportPos[ii].azimuth;// counter clockwise angle between this robot shooter and the obstacle

                // cam 1 is left of cam 0 = counterclockwise
                angleRad += camIndex * CV_PI / 2.0;            // camIndex * 90 degrees, every camera is 90 degrees rotated

                // from center of this robot to closest edge of ball, there is no need to add a ballRadius to radius (ball edge) because in field view the radius is very close to the center
                // exportPos[ii].radius is in mm, while Ros expects meters
                double radius = 0.001f * exportPos[ii].radius;
                double score = size / 1000.0f;
                if (score > 1.0) {
                    score = 1.0;
                }
                // cout << "ball index " << ii << " size " << size << " score " << score << " << " angle radians " << angleRad << " radius " << radius << endl;

                obstaclePositionType obstacle;

                obstacle.setAngle(angleRad);
                obstacle.setRadius(radius);
                obstacle.setConfidence(score);
                int color = 0;                    // 0 is black
                obstacle.setColor(color);

                obstacles.push_back(obstacle);
            }
        }

        for (vector<observer*>::const_iterator iter = vecObservers.begin(); iter != vecObservers.end(); ++iter) {
            if (*iter != NULL) {
                (*iter)->update_own_obstacle_position(obstacles, conf->getObstacleLatencyOffset());
            }
        }
    }

#endif
}

void obstacleDetection::attach(observer *observer) {
    vecObservers.push_back(observer);
}

void obstacleDetection::detach(observer *observer) {
    vecObservers.erase(std::remove(vecObservers.begin(), vecObservers.end(), observer), vecObservers.end());
}

std::vector<linePointSt> obstacleDetection::getObstaclePoints() {
    obstaclePointListExportMutex.lock();
    std::vector<linePointSt> retVal = obstaclePointList;
    obstaclePointListExportMutex.unlock();
    return retVal;
}
