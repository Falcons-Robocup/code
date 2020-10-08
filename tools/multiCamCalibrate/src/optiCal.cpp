/*
 * optiCal.cpp
 *
 * Calibration sequences.
 *
 *  Created on: May 2018
 *      Author: Jan Feitsma & Serhat Colak
 */


#include <stdexcept>
#include <unistd.h>
#include "optiCal.hpp"
#include "CameraCalibrator.hpp"


void opticalCalibrator::calibrateFisheye()
{
    // this function is supposed to run in its own thread, next to GUI showing video feed
    int n = 0;
    std::stringstream strForm;
    n = FISHEYE_WAIT;
    while (n--)
    {
        strForm.str("");
        strForm << "fisheye: starting in " << n << " second(s) ...";
        setGuiText(strForm.str());
        sleep(1);
    }
    n = FISHEYE_NUM_FRAMES;
    std::vector<cv::Mat> images;
    while (n--)
    {
        strForm.str("");
        strForm << "fisheye: " << n << " images remaining ...";
        setGuiText(strForm.str());
        images.push_back(getCamFrame());
        sleep(1);
    }
    // calibrate to find camera parameters
    CameraCalibrator cameraCalibrator;
    cv::Matx33d K;
    cv::Vec4d D;
    setGuiText("calibrating fisheye ...");
    cameraCalibrator.run(images, K, D);
    {
        std::unique_lock<std::mutex> lck(_mtx);
        _calData.K = K;
        _calData.D = D;
        _calData.fisheyeCalibrated = true;
    }
    cv::fisheye::initUndistortRectifyMap(K, D, cv::Matx33d::eye(), K, images.back().size(), CV_32FC1, _rmapx, _rmapy);
    setView(1); // will notify user
}

void opticalCalibrator::calibratePerspectiveClicks()
{
    // this function is supposed to run in its own thread, next to GUI showing video feed
    // ask user to identify a number of landmarks
    // TODO: detect by algorithm, to minimize user intervention and increase accuracy
    clearClicks();
    auto clicks = getClicks();
    int remaining = PERSPECTIVE_NUM_POINTS - clicks.size();
    while (remaining > 0)
    {
        std::stringstream strForm;
        strForm << "identify landmark (" << _landmarksFcs[4-remaining].x << "," << _landmarksFcs[4-remaining].y << ") ...";
        setGuiText(strForm.str());
        sleep(1);
        clicks = getClicks();
        remaining = PERSPECTIVE_NUM_POINTS - clicks.size();
    }
    calibratePerspectiveCalculate();
}

void opticalCalibrator::calibratePerspectiveCalculate()
{
    // get homography
    std::stringstream strForm;
    strForm << "calculating maps ...";
    setGuiText(strForm.str());
    cv::Mat H = cv::findHomography(getClicks(), _landmarksRcsMM);
    {
        std::unique_lock<std::mutex> lck(_mtx);
        _calData.Hf = H;
        _calData.perspectiveCalibrated = true;
    }
    // invalidate floor maps
    _floorMapsValid = false;
    setView(2); // will notify user
}

cv::Mat opticalCalibrator::getHV()
{
    // Hv is the homography transformation to make the floor lookup map fit nicely in the GUI
    // robot is middle bottom, 1 meter is a certain number of pixels (ppm)
    // this is a static map, so we calculate when first requested, then cache
    if (_Hv.empty())
    {
        cv::Mat frame = removeBorder(getCamFrame());
        float ppm = PIXELS_PER_METER * (1+getZoom());
        float m = 1e3; // from millimeters
        std::vector<cv::Point2f> A;
        std::vector<cv::Point2f> B;
        A.push_back(cv::Point2f(0, 0)); B.push_back(cv::Point2f(frame.cols * 0.5, frame.rows));
        A.push_back(cv::Point2f(0, m)); B.push_back(cv::Point2f(frame.cols * 0.5, frame.rows - ppm));
        A.push_back(cv::Point2f(m, 0)); B.push_back(cv::Point2f(frame.cols * 0.5 + ppm, frame.rows));
        A.push_back(cv::Point2f(m, m)); B.push_back(cv::Point2f(frame.cols * 0.5 + ppm, frame.rows - ppm));
        _Hv = cv::findHomography(A, B);
    }
    return _Hv;
}

void opticalCalibrator::ensureFloorMaps()
{
    if (_floorMapsValid == false)
    {
        calcFloorMaps();
    }
}

void opticalCalibrator::ensureFrontMaps()
{
    if (_frontMapsValid == false)
    {
        calcFrontMaps();
    }
}

void opticalCalibrator::calcFloorMaps()
{
    // initialize
    cv::Mat frame = getCamFrame();
    int ncol = frame.cols;
    int nrow = frame.rows;
    _floorMapX = cv::Mat(nrow, ncol, CV_16SC1, cv::Scalar(0));
    _floorMapY = cv::Mat(nrow, ncol, CV_16SC1, cv::Scalar(0));
    cv::Mat count(500, 1000, CV_16SC1, cv::Scalar(0));
    // make list of coordinates
    std::vector<cv::Point2f> inputPoints;
    for (int irow = 0; irow < nrow; ++irow)
    {
        for (int icol = 0; icol < ncol; ++icol)
        {
            inputPoints.push_back(cv::Point2f((float)icol, (float)(irow)));
        }
    }
    // correct for fisheye
    std::vector<cv::Point2f> pointsUnwarped;
    cv::fisheye::undistortPoints(inputPoints, pointsUnwarped, _calData.K, _calData.D, cv::Matx33d::eye(), _calData.K);
    // apply homography transform
    std::vector<cv::Point2f> pointsResult;
    perspectiveTransform(pointsUnwarped, pointsResult, _calData.Hf);
    // put points in lookup map
    for (size_t idx = 0; idx < pointsResult.size(); ++idx)
    {
        int xi = (int)inputPoints[idx].x;
        int yi = (int)inputPoints[idx].y;
        int xo = (int)pointsResult[idx].x; // in mm RCS
        int yo = (int)pointsResult[idx].y; // in mm RCS
        float r = 1e-3 * sqrt(xo*xo + yo*yo);
        if (yo >= 0 && r > MIN_CUTOFF_RADIUS && r < MAX_CUTOFF_RADIUS && fabs(1e-3*xo) < MAX_CUTOFF_RADIUS && 1e-3*yo < MAX_CUTOFF_RADIUS)
        {
            // TODO select BEST point?
            if (xi >= 0 && xi < ncol && yi >= 0 && yi < nrow)
            {
                _floorMapX.at<int16_t>(yi, xi) = (int16_t)xo;
                _floorMapY.at<int16_t>(yi, xi) = (int16_t)yo;
                int xc = (int16_t)(xo*0.1);
                int yc = (int16_t)(yo*0.1);
                if (yc < count.rows && fabs(xc) < count.cols * 0.5)
                    count.at<int16_t>(yc, xc+500) += 1;
            }
        }
    }
    /*
    // debug: dump to file?
    FILE *fp = fopen("/var/tmp/counts.txt", "w");
    for (int irow = 0; irow < count.rows; ++irow)
    {
        for (int icol = 0; icol < count.cols; ++icol)
        {
            int c = count.at<int16_t>(irow, icol);
            if (c > 9) c = 9;
            if (c > 0)
                fprintf(fp, "%d", c);
            else
                fprintf(fp, " ");

        }
        fprintf(fp, "\n");
    }
    fclose(fp);
    */
    _floorMapsValid = true;
}

std::vector<cv::Point2f> opticalCalibrator::landmarksFront()
{
    // use a virtual plane 1m in front of the robot, i.e. RCS.y == 1
    std::vector<cv::Point2f> landmarksInFrontCoordinateSystem;
    float H = 0.75; // nominal camera mounting height -- TODO make configurable, or at least store in .bin
    int cameraId = 0;
    Position2D robotPosFcs(0.00, 0.00, M_PI*0.25 + M_PI*0.5*cameraId);
    for (auto it = _landmarksFcs.begin(); it != _landmarksFcs.end(); ++it)
    {
        Position2D landMarkRCS = Position2D(*it).transform_fcs2rcs(robotPosFcs);
        //float d = sqrt(landMarkRCS.y * landMarkRCS.y + landMarkRCS.x * landMarkRCS.x);
        float x = landMarkRCS.x / landMarkRCS.y;
        float z = -H / landMarkRCS.y;
        landmarksInFrontCoordinateSystem.push_back(cv::Point2f(x, z));
    }
    return landmarksInFrontCoordinateSystem;
}

void opticalCalibrator::calcFrontMaps()
{
    // initialize
    cv::Mat frame = getCamFrame();
    int ncol = frame.cols;
    int nrow = frame.rows;
    _frontMapAz = cv::Mat(nrow, ncol, CV_32FC1, cv::Scalar(0.0));
    _frontMapEl = cv::Mat(nrow, ncol, CV_32FC1, cv::Scalar(0.0));
    // make list of coordinates
    std::vector<cv::Point2f> inputPoints;
    for (int irow = 0; irow < nrow; ++irow)
    {
        for (int icol = 0; icol < ncol; ++icol)
        {
            inputPoints.push_back(cv::Point2f((float)icol, (float)(irow)));
        }
    }
    // correct for fisheye
    std::vector<cv::Point2f> pointsUnwarped;
    cv::fisheye::undistortPoints(inputPoints, pointsUnwarped, _calData.K, _calData.D, cv::Matx33d::eye(), _calData.K);
    // apply front homography transform
    auto clicks = getClicks();
    if (clicks.empty())
    {
        // reconstruct clicks from floor homography
        // TODO: store in calibration data
        perspectiveTransform(_landmarksRcsMM, clicks, _calData.Hf.inv());
        //for (auto it = clicks.begin(); it != clicks.end(); ++it)
        //{
        //    printf("reconstructed click %4d %4d\n", (int)it->x, (int)it->y);
        //}
    }
    cv::Mat Hfront = cv::findHomography(clicks, landmarksFront());
    std::vector<cv::Point2f> pointsResult;
    perspectiveTransform(pointsUnwarped, pointsResult, Hfront);
    // put points in lookup map
    for (size_t idx = 0; idx < pointsResult.size(); ++idx)
    {
        int xi = (int)inputPoints[idx].x;
        int yi = (int)inputPoints[idx].y;
        float az = atan(pointsResult[idx].x);
        float el = atan(pointsResult[idx].y);
        _frontMapAz.at<float>(yi, xi) = az;
        _frontMapEl.at<float>(yi, xi) = el;
    }

    _frontMapsValid = true;
}

cv::Mat opticalCalibrator::transformVerification(cv::Mat const &frame)
{
    // calculate maps if not done already
    ensureFloorMaps();
    // initialize as black image
    cv::Mat result = cv::Mat(frame.rows, frame.cols, frame.type(), cv::Scalar(0));
    cv::Mat Hv = getHV();
    // fill result image
    for (int irow = BORDER_SIZE; irow < frame.rows - BORDER_SIZE; ++irow)
    {
        for (int icol = 0; icol < frame.cols; ++icol)
        {
            // like in per-point dewarp::transform
            int x = _floorMapX.at<int16_t>(irow, icol);
            int y = _floorMapY.at<int16_t>(irow, icol);
            // apply perspective transform to map to GUI window
            std::vector<cv::Point2f> vecRcsMM;
            std::vector<cv::Point2f> vecView;
            vecRcsMM.push_back(cv::Point2f(x, y));
            cv::perspectiveTransform(vecRcsMM, vecView, Hv);
            x = (int)vecView[0].x;
            y = (int)vecView[0].y;
            if ((y >= 0) && (y < frame.rows) && (x >= 0) && (x < frame.cols))
            {
                result.at<cv::Vec3b>(y, x) = frame.at<cv::Vec3b>(irow, icol);
            }
        }
    }
    return result;
}

void opticalCalibrator::upgrade()
{
    ensureFloorMaps();
    ensureFrontMaps();
}

