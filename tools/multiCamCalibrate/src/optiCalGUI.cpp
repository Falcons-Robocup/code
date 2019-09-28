/*
 * optiCalGUI.cpp
 *
 * Optical Calibration: GUI logic and utility functions.
 * 
 *  Created on: May 2018
 *      Author: Jan Feitsma
 */


#include <stdexcept>
#include <unistd.h>
#include <ctime>
#include "optiCal.hpp"
#include "cvmatio.hpp"
#include "CameraCalibrator.hpp"


  
opticalCalibrator::opticalCalibrator()
{
    _guiText = "uncorrected";
    _windowName = "optiCal";
    calcLandMarks();
}
    
opticalCalibrator::~opticalCalibrator()
{
    if (_workerThread != NULL)
    {
        if (_workerThread->joinable())
        {
            _workerThread->join();
        }
        delete _workerThread;
    }
    // TODO: graceful shutdown & interrupt
    // for now it's ok as long as you do not try to close the application while workerThread is busy
}
    
void opticalCalibrator::run()
{
    // check that we have a valid feed
    if (_videoFeed == NULL)
    {
        throw std::runtime_error("no video feed configured");
    }
    // GUI loop
    for (;;)
    {
        // get a new frame from camera (or still, or ..)
        updateCamFrame();
        // show
        showFrame();
        int key = cv::waitKeyEx(30);
        if (key >= 0)
        {
            if ((key == 27) || (key == 'q')) 
            {
                // TODO: introduce a shutdown state, to also make worker threads quit immediately
                break; // escape / quit
            }
            else
            {
                handleKeyPress(key);
            }
        }
    }
}

void opticalCalibrator::connectVideo(multiCamVideoFeed *videoFeed)
{
    if (videoFeed != NULL)
    {
        _videoFeed = videoFeed;
    }
}

cv::Mat opticalCalibrator::addBorder(cv::Mat const &frame)
{
    cv::Mat result;
    cv::copyMakeBorder(frame, result, BORDER_SIZE, BORDER_SIZE, 0, 0, cv::BORDER_CONSTANT);
    return result;
}

cv::Mat opticalCalibrator::removeBorder(cv::Mat const &frame)
{
    cv::Rect roi;
    roi.x = 0;
    roi.y = BORDER_SIZE;
    roi.width = frame.size().width;
    roi.height = frame.size().height - (BORDER_SIZE * 2);
    cv::Mat result = frame(roi);
    return result;
}

void opticalCalibrator::showFrame()
{
    // start with latest raw camera frame
    cv::Mat inputFrame = getCamFrame();
    cv::Mat frame = inputFrame.clone();
    int view = getView();
    // apply transformations
    if ((view == 1) || (view == 2))
    {
        // fisheye correction
        cv::remap(inputFrame, frame, _rmapx, _rmapy, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    }
    if (view == 2)
    {
        // perspective transform
        cv::Mat tmpFrame = frame; // warpPerspective cannot operate in-place
        warpPerspective(tmpFrame, frame, getHV() * _calData.Hf, cv::Size(frame.cols, frame.rows));
    }
    if (view == 3)
    {
        // verification: transform each pixel using lookup maps, as used in realtime localization code
        // include view homography to make raw lookup map viewable
        frame = transformVerification(inputFrame);
    }
    // put text, lines and stuff on top of the camera frame
    cv::Mat frameWithExtras = frame.clone();
    cv::putText(frameWithExtras, getGuiText(), cv::Point(10,30), CV_FONT_NORMAL, 1, cv::Scalar(0, 0, 255), 2);
    if (view >= 2)
    {
        drawGrid(frameWithExtras);
        drawLandMarks(frameWithExtras);
    }
    // show in window
    cv::imshow(_windowName, frameWithExtras);
}

void opticalCalibrator::drawLandMarks(cv::Mat &frame)
{
    cv::Mat Hv = getHV();
    std::vector<cv::Point2f> pts;
    cv::perspectiveTransform(_landmarksRcsMM, pts, Hv);
    for (size_t idx = 0; idx < pts.size(); ++idx)
    {
        auto color = CV_RGB(0, 0, 255);
        if (_selectedLandmark-1 == (int)idx)
        {
            color = CV_RGB(255, 0, 255);
        }
        circle(frame, pts[idx], 1, color, 5);
    }
}

void opticalCalibrator::drawGrid(cv::Mat &frame)
{
    int MX = 10;
    int MY = 10;
    cv::Mat Hv = getHV();
    // green 1m-per-square grid
    for (int rx = -MX; rx <= MX; ++rx)
    {
        std::vector<cv::Point2f> iPts, oPts;
        iPts.push_back(cv::Point2f(rx/MM, 0));
        iPts.push_back(cv::Point2f(rx/MM, MY/MM));
        cv::perspectiveTransform(iPts, oPts, Hv);
		cv::line(frame, oPts[0], oPts[1], CV_RGB(0, 255, 0), 1);
    }
    for (int ry = 0; ry <= MY; ++ry)
    {
        std::vector<cv::Point2f> iPts, oPts;
        iPts.push_back(cv::Point2f(-MX/MM, ry/MM));
        iPts.push_back(cv::Point2f(MX/MM, ry/MM));
        cv::perspectiveTransform(iPts, oPts, Hv);
		cv::line(frame, oPts[0], oPts[1], CV_RGB(0, 255, 0), 1);
    }
    // red diagonals
    for (int sign = -1; sign <= 1; sign += 2)
    {
        std::vector<cv::Point2f> iPts, oPts;
        iPts.push_back(cv::Point2f(sign*MX/MM, MY/MM));
        iPts.push_back(cv::Point2f(0, 0));
        cv::perspectiveTransform(iPts, oPts, Hv);
		cv::line(frame, oPts[0], oPts[1], CV_RGB(255, 0, 0), 1);
    }
    // red field boundaries
    for (size_t it = 1; it < _boundariesRcsMM.size(); ++it)
    {
        std::vector<cv::Point2f> iPts, oPts;
        iPts.push_back(cv::Point2f(_boundariesRcsMM[it].x, _boundariesRcsMM[it].y));
        iPts.push_back(cv::Point2f(_boundariesRcsMM[it-1].x, _boundariesRcsMM[it-1].y));
        cv::perspectiveTransform(iPts, oPts, Hv);
		cv::line(frame, oPts[0], oPts[1], CV_RGB(255, 0, 0), 1);
    }
}

std::string opticalCalibrator::getGuiText()
{
    std::unique_lock<std::mutex> lck(_mtx);
    return _guiText;
}
    
void opticalCalibrator::setGuiText(std::string const &txt)
{
    std::unique_lock<std::mutex> lck(_mtx);
    _guiText = txt;
}

void opticalCalibrator::setView(int view)
{
    {
        std::unique_lock<std::mutex> lck(_mtx);
        // only if calibration state is sufficient
        if ((view > 0) && (view < 3) && (_calData.fisheyeCalibrated == false)) 
        {
            std::cerr << "require fisheye calibration" << std::endl;
            return;
        }
        if ((view == 2) && (_calData.perspectiveCalibrated == false)) 
        {
            std::cerr << "require perspective calibration" << std::endl;
            return;
        }
        // ok to update
        _view = view;
        // release lock
    }
    // notify user
    switch (_view)
    {
      case (0):
        setGuiText(_videoFeed->describe());
        break;
      case (1):
        setGuiText("corrected for fisheye");
        break;
      case (2):
        setGuiText("corrected for perspective");
        break;
      case (3):
        setGuiText("per-pixel verification");
        break;
    }
}

int opticalCalibrator::getView()
{
    std::unique_lock<std::mutex> lck(_mtx);
    return _view;
}

cv::Mat opticalCalibrator::getCamFrame()
{
    std::unique_lock<std::mutex> lck(_mtx);
    return _camFrame;
}
    
void opticalCalibrator::updateCamFrame()
{
    std::unique_lock<std::mutex> lck(_mtx);
    if (_videoFeed != NULL)
    {
        _camFrame = _videoFeed->getFrame();
        // use a temporary border to prevent the high warpage from cropping too much!!
        // TODO: very error prone ... make code nicer & safer
        _camFrame = addBorder(_camFrame);
    }
}

void opticalCalibrator::clearClicks()
{
    std::unique_lock<std::mutex> lck(_mtx);
    _clicks.clear();
}

std::vector<cv::Point2f> opticalCalibrator::getClicks()
{
    std::unique_lock<std::mutex> lck(_mtx);
    return _clicks;
}

void opticalCalibrator::addClick(int x, int y)
{
    std::unique_lock<std::mutex> lck(_mtx);
    _clicks.push_back(cv::Point2f(x, y));
}

void opticalCalibrator::toggleCamera()
{
    _cameraId = (_cameraId + 1) % 4;
    setGuiText("selecting camera " + std::to_string(_cameraId));
    if (_videoFeed != NULL)
    {
        _videoFeed->selectInput(_cameraId);
        _videoFeed->setMode(videoInputMode::MULTICAM);
    }
    calcLandMarks();
}

void opticalCalibrator::selectCamera(int cameraId)
{
    _cameraId = cameraId;
    calcLandMarks();
}

void opticalCalibrator::calcLandMarks()
{
    _landmarksFcs.clear();
    float sy = ((_cameraId < 2) ? 1.0 : -1.0);
    float sx = ((_cameraId == 0 || _cameraId == 3) ? 1.0 : -1.0);
    // accuracy is critical between 2 and 6 meters
    // since robot is positioned on (0,0), we use corresponding landmarks (might need some black markers on the field for (0,+/-2))
    // using only x- and y- depth makes fine-tuning also much easier
    _landmarksFcs.push_back(Position2D( sx*2.00,  sy*0.00, 0));
    _landmarksFcs.push_back(Position2D( sx*6.00,  sy*0.00, 0));
    _landmarksFcs.push_back(Position2D( sx*0.00,  sy*2.00, 0));
    _landmarksFcs.push_back(Position2D( sx*0.00,  sy*6.00, 0));
    // TODO: extract cornerpos and landmarks from common field configuration instead of hardcoding
    // define the landmarks w.r.t. robot position, which should be facing diagonally into the field from left-back corner, Locht field configuration
    // NOTE: robot should be placed quite accurately, using the wooden 'cape', such that the black diagonals line up with outside of white lines
    // NOTE: it should not be needed to apply fine-tuning (Andre's yaml setting)
    Position2D robotPosFcs(0.00, 0.00, M_PI*0.25 + M_PI*0.5*_cameraId);
    // the homography map and the related forward maps are defined in RCS millimeters
    // for visualization however, we need to shift and scale, to produce sensible plots
    _landmarksRcsMM.clear();
    for (auto it = _landmarksFcs.begin(); it != _landmarksFcs.end(); ++it)
    {
        Position2D landMarkRCS = Position2D(*it).transform_fcs2rcs(robotPosFcs);
        _landmarksRcsMM.push_back(cv::Point2f((int)(landMarkRCS.x / MM), (int)(landMarkRCS.y / MM)));
    }
    // invalidate any existing homography maps
    _calData.perspectiveCalibrated = false;
    clearClicks();
    // for grid drawing (red lines)
    std::vector<Position2D> boundariesFCS;
    boundariesFCS.push_back(Position2D(    0.00,  sy*9.00, 0));
    boundariesFCS.push_back(Position2D( sx*6.00,  sy*9.00, 0));
    boundariesFCS.push_back(Position2D( sx*6.00,     0.00, 0));
    _boundariesRcsMM.clear();
    for (auto it = boundariesFCS.begin(); it != boundariesFCS.end(); ++it)
    {
        Position2D tmp = Position2D(*it).transform_fcs2rcs(robotPosFcs);
        _boundariesRcsMM.push_back(cv::Point2f((int)(tmp.x / MM), (int)(tmp.y / MM)));
    }
}

void opticalCalibrator::startThread(std::thread *t)
{
    _workerThread = t;
}

void opticalCalibrator::handleKeyPress(int key)
{
    switch (key)
    {
      case ('h'):
        showHelp();
        break;
      case ('f'):
        startThread(new std::thread(&opticalCalibrator::calibrateFisheye, this));
        break;
      case ('p'):
        cv::setMouseCallback(_windowName, onMouse, this);
        startThread(new std::thread(&opticalCalibrator::calibratePerspectiveClicks, this));
        break;
      case ('r'):
        setView(0);
        break;
      case ('w'):
        setView(1);
        break;
      case ('g'):
        setView(2);
        break;
      case ('v'):
        setView(3);
        break;
      case ('s'):
        save();
        break;
      case ('z'):
        toggleZoom();
        break;
      case ('c'):
        toggleCamera();
        break;
      case (','): // left
        moveClick(-1, 0);
        break;
      case ('.'): // up
        moveClick(0, 1);
        break;
      case ('/'): // right
        moveClick(1, 0);
        break;
      case ('m'): // down
        moveClick(0, -1);
        break;
      case ('0'):
      case ('1'):
      case ('2'):
      case ('3'):
      case ('4'):
        _selectedLandmark = key - '0';
        break;
      case ('i'):
        cv::imwrite("/var/tmp/optiCalFrame.png", removeBorder(getCamFrame()));
        break;
    }
}

void opticalCalibrator::moveClick(int dx, int dy)
{
    {
        std::unique_lock<std::mutex> lck(_mtx);
        float scale = 0.5;
        if ((_selectedLandmark > 0) && (_clicks.size() == 4))
        {
            _clicks[_selectedLandmark-1].x += dx * scale;
            _clicks[_selectedLandmark-1].y += dy * scale;
        }
    }
    calibratePerspectiveCalculate();
}

bool opticalCalibrator::getZoom()
{
    std::unique_lock<std::mutex> lck(_mtx);
    return _zoom;
}

void opticalCalibrator::toggleZoom()
{
    std::unique_lock<std::mutex> lck(_mtx);
    _Hv = cv::Mat(); // reinit to empty to force recalc
    _zoom = !_zoom;
}

void opticalCalibrator::showHelp()
{
    std::cout << " h      : show this help" << std::endl;
    std::cout << " f      : calibrate fisheye using the portable chessboard (phase 1)" << std::endl;
    std::cout << " p      : calibrate perspective transform using landmarks on the field (phase 2)" << std::endl;
    std::cout << " 0..4   : select a landmark (0 to deselect)" << std::endl;
    std::cout << " ,./m   : move selected landmark (arrows key don't work ...)" << std::endl;
    std::cout << " r      : show raw camera feed" << std::endl;
    std::cout << " w      : show fisheye-corrected camera feed" << std::endl;
    std::cout << " g      : show resulting feed with verification grid" << std::endl;
    std::cout << " v      : verify mode by translating all pixels individually" << std::endl;
    std::cout << " z      : zoom toggle (only applies to grid- and verify view mode)" << std::endl;
    std::cout << " c      : cam toggle (0,1,2,3)" << std::endl;
    std::cout << " s      : save calibration to file" << std::endl;
    std::cout << " i      : save raw camera frame to file" << std::endl;
}

// TODO: load and save are not very robust, be sure to use correct data

void opticalCalibrator::load(std::string filename)
{
    // NOTE: this code is highly duplicate and depending on dewarp.cpp
    std::ifstream ifs(filename, std::ios::binary);
    // header: image dimensions
    int ncol, nrow;
    ifs.read((char*)&ncol, sizeof(ncol));
    ifs.read((char*)&nrow, sizeof(nrow));
    // x and y lookups
    readMatBinary(ifs, _fmapx);
    readMatBinary(ifs, _fmapy);
    // calibration struct
    readMatBinary(ifs, _calData.K);
    readMatBinary(ifs, _calData.D);
    readMatBinary(ifs, _calData.Hf);
    _calData.fisheyeCalibrated = true;
    _calData.perspectiveCalibrated = true;
    // construct reverse maps
    cv::fisheye::initUndistortRectifyMap(_calData.K, _calData.D, cv::Matx33d::eye(), _calData.K, cv::Size(ncol, nrow+2*BORDER_SIZE), CV_32FC1, _rmapx, _rmapy);
    // pre-calculate forward lookups
    // calcForwardMaps(); no, we might nog have an image yet! (still mode)
}

void opticalCalibrator::save(std::string filename)
{
    if (filename == "")
    {
        // automatically determine output filename
        time_t rawtime;
        struct tm *timeinfo;
        char buffer[80];
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        std::string s = "r" + std::to_string(getRobotNumber()) + "_cam" + std::to_string(_cameraId);
        strftime(buffer, sizeof(buffer), ("/var/tmp/%Y%m%d_%H%M%S_" + s + ".bin").c_str(), timeinfo);
        filename = buffer;
    }
    // calculate maps if not done already
    ensureForwardMaps();
    // save
    // NOTE: this code is highly duplicate and depending on dewarp.cpp
    std::ofstream ofs(filename, std::ios::binary);
    // header: image dimensions
    cv::Mat frame = removeBorder(getCamFrame());
    ofs.write((char*)&frame.cols, sizeof(frame.cols));
    ofs.write((char*)&frame.rows, sizeof(frame.rows));
    // x and y lookups
    writeMatBinary(ofs, _fmapx);
    writeMatBinary(ofs, _fmapy);
    // calibration struct
    writeMatBinary(ofs, _calData.K);
    writeMatBinary(ofs, _calData.D);
    writeMatBinary(ofs, _calData.Hf);
}

void opticalCalibrator::onMouse(const int event, const int x, const int y, int, void* userdata)
{
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        opticalCalibrator *self = (opticalCalibrator *)userdata;
        self->addClick(x, y);
    }
}

