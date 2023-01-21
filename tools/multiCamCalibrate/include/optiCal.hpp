/*
 * optiCal.hpp
 *
 *  Created on: May, 2018
 *      Author: Jan Feitsma
 */

#ifndef OPTICAL_HPP_
#define OPTICAL_HPP_

#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <string>

#define WITHOUT_RTDB // workaround for an undesired dependency from vector3d.hpp to RTDB ...
#include "falconsCommon.hpp"
#undef WITHOUT_RTDB

#include "multiCamVideoFeed.hpp"

#define FISHEYE_WAIT 5
#define FISHEYE_NUM_FRAMES 20
#define PERSPECTIVE_NUM_POINTS 4
#define MM 1e-3
#define MAX_CUTOFF_RADIUS 8
#define MIN_CUTOFF_RADIUS 0.1
#define BORDER_SIZE 120
#define PIXELS_PER_METER 50


struct calData
{
    cv::Matx33d K; // fisheye
    cv::Vec4d D; // fisheye
    bool fisheyeCalibrated = false;
    cv::Mat Hf; // homography corresponding to coordinates in RCS in millimeters, as used in lookup maps
    bool perspectiveCalibrated = false;
};


class opticalCalibrator
{

public:
    opticalCalibrator();
    ~opticalCalibrator();
    void load(std::string filename);
    void save(std::string filename = "");
    void upgrade();
    void selectCamera(int cameraId);
    void connectVideo(multiCamVideoFeed *videoFeed);
    void run();
    void setPylon();
    
private:
    multiCamVideoFeed *_videoFeed = NULL;
    std::string  _guiText;
    std::string  _windowName;
    cv::Mat      _camFrame;
    std::mutex   _mtx;
    std::thread *_workerThread = NULL;
    calData      _calData;
    cv::Mat      _rmapx;
    cv::Mat      _rmapy;
    cv::Mat      _floorMapX;
    cv::Mat      _floorMapY;
    cv::Mat      _frontMapAz;
    cv::Mat      _frontMapEl;
    cv::Mat      _Hv; // homography matrix to make the floor map viewable in GUI
    int          _view = 0;
    bool         _zoom = false;
    bool         _floorMapsValid = false;
    bool         _frontMapsValid = false;
    int          _cameraId = 0; // 0, 1, 2 or 3
    std::vector<cv::Point2f> _clicks;
    std::vector<Position2D>  _landmarksFcs; // phi unused
    std::vector<cv::Point2f> _landmarksRcsMM;
    int          _selectedLandmark = 0;
    std::vector<cv::Point2f> _boundariesRcsMM; // for grid drawing
    bool         _usePylonCam;

    // main functionality: implementations in optiCal.cpp
    void calibrateFisheye();
    void calibratePerspectiveClicks();
    void calibratePerspectiveCalculate();
    void calcFloorMaps();
    void calcFrontMaps();
    cv::Mat transformVerification(cv::Mat const &frame);
    cv::Mat getHV();
    std::vector<cv::Point2f> landmarksFront();

    // secondary functionality: implementations in optiCalGUI.cpp
    std::string getGuiText();
    cv::Mat getCamFrame();
    void setGuiText(std::string const &txt);
    void updateCamFrame();
    void startThread(std::thread *t);
    void showFrame(); 
    void handleKeyPress(int key);
    void clearClicks();
    std::vector<cv::Point2f> getClicks();
    void addClick(int x, int y);
    void moveClick(int dx, int dy);
    void setView(int view);
    int getView();
    bool getZoom();
    void toggleZoom();
    void toggleCamera();
    void calcLandMarks();
    void ensureFloorMaps();
    void ensureFrontMaps();
    cv::Mat addBorder(cv::Mat const &frame);
    cv::Mat removeBorder(cv::Mat const &frame);
    void drawGrid(cv::Mat &frame);
    void drawLandMarks(cv::Mat &frame);
    void showHelp();
    static void onMouse(int event, int x, int y, int flags, void* userdata);
};

#endif /* OPTICAL_HPP_ */

