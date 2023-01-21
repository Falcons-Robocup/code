/*
 * multiCamVideoFeed.cpp
 *
 *  Created on: May, 2018
 *      Author: Jan Feitsma
 */


#include "multiCamVideoFeed.hpp"


multiCamVideoFeed::multiCamVideoFeed()
{
    // create the camera system receive that will receive also the jpg from the raspi camera's
    _camSysRecv = new camSysReceive();
    _camSysRecv->setImageGrabPath("."); // do not use "/dev/shm" which is used by multiCam

    // start thread that collects data from multiple camera's
    _camSysRecvThread = std::thread(&camSysReceive::receive, _camSysRecv);
}

multiCamVideoFeed::~multiCamVideoFeed()
{
}

std::string multiCamVideoFeed::describe()
{
    std::string result;
    switch (_mode)
    {
      case videoInputMode::NONE:
        throw std::runtime_error("no source selected");
        break;
      case videoInputMode::STILL:
        result = "static image";
        break;
      case videoInputMode::USB:
        result = "USB video feed";
        break;
      case videoInputMode::MULTICAM:
        result = "multiCam " + std::to_string(_inputSelector);
        break;
      case videoInputMode::PYLONCAM:
        result = "pylonCam " + std::to_string(_inputSelector);
        break;
    }
    return result;
}

void multiCamVideoFeed::setMode(videoInputMode const &mode)
{
    _mode = mode;
}

void multiCamVideoFeed::setStill(std::string const &filename)
{
    _overrideFile = filename;
    _overrideFrame = cv::imread(_overrideFile);
    setMode(videoInputMode::STILL);
}

void multiCamVideoFeed::setUsb(int device)
{
    _mode = videoInputMode::USB;
    // connect to /dev/videoX
    _capture = new cv::VideoCapture();
    _capture->open(device);
    if ( !_capture->isOpened() ) {
       printf("ERROR     : unable to open usb camera /dev/video%d\n", device);
    } else {
       printf("INFO      : usb camera /dev/video%d is ready to use\n", device);
    }
    // for human recognition test, the genius f100 is set to 1280 x 720 (of which we only use 960 x 720)
    _capture->set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    _capture->set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    printf("INFO      : crop and resize 1280x720 to 608x800 (to match with raspi camera)\n");
}

void multiCamVideoFeed::setPylon()
{
    _mode = videoInputMode::PYLONCAM;

    for( size_t ii = 0; ii < 4; ii++ ) {
        _pylonGrab[ii] = new grabber();
        _pylonGrab[ii]->attach(ii);
        printf("INFO    serial of index %zu camera is %s\n", ii, _pylonGrab[ii]->getSerial().c_str());
        _pylonGrab[ii]->configure();
        _pylonGrab[ii]->start();

        // start thread that receives the images from the selected camera
        _pylonGrabThread[ii] = std::thread(&grabber::update, _pylonGrab[ii]);
    }
}

void multiCamVideoFeed::selectInput(int input)
{
    _inputSelector = input;

    // the order of the camera's is determined by the serial, but this does not match with the mechanical mount
    // swap camera 1 and 3 to match with mechanical order
    // TODO: find better way of alignment between serial and mechanical order
    // WARNING: it might be that now the configuration of the camera does not match anymore with the video data
    int cam = _inputSelector;
    if( cam == 1 ) {
       cam = 3;
    } else if( cam == 3 ) {
       cam = 1;
    }

    if( _mode == videoInputMode::PYLONCAM ) {
       printf("INFO    serial of index %d camera is %s\n", input, _pylonGrab[cam]->getSerial().c_str());
    }
}

cv::Mat multiCamVideoFeed::getFrame()
{
    cv::Mat result;
    cv::Mat tmpFrame;

    // Andre: to have a better match with the training data (created from portrait 608x800)
    // reduce the 1280x720 wide angle to 960x720
    int width = 960;
    cv::Rect cropRegion((1280-width)/2, 0, width, 720);

    switch (_mode)
    {
      case videoInputMode::NONE:
        throw std::runtime_error("no source selected");
        break;
      case videoInputMode::STILL:
        result = _overrideFrame;
        break;
      case videoInputMode::USB:
        if (_capture != NULL)
        {
            (*_capture) >> tmpFrame;
            // crop the captured frame
            tmpFrame = tmpFrame(cropRegion);
            cv::resize(tmpFrame, tmpFrame, {608, 800}, 0, 0, cv::INTER_LINEAR);
            cv::rotate(tmpFrame, tmpFrame, cv::ROTATE_90_CLOCKWISE);
            result = tmpFrame;
        }
        break;
      case videoInputMode::MULTICAM:
         _camSysRecv->getCameraFrame(_inputSelector).copyTo(result);
         break;
      case videoInputMode::PYLONCAM:
         int cam = _inputSelector;
         if( cam == 1 ) {
            cam = 3;
         } else if( cam == 3 ) {
            cam = 1;
         }
         _pylonGrab[cam]->getImage().copyTo(tmpFrame);
         cv::rotate(tmpFrame, tmpFrame, cv::ROTATE_90_CLOCKWISE);
         result = tmpFrame;
         break;
    }
    return result;
}

