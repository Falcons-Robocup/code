/*
 * multiCamVideoFeed.cpp
 *
 *  Created on: May, 2018
 *      Author: Jan Feitsma
 */


#include "multiCamVideoFeed.hpp"


multiCamVideoFeed::multiCamVideoFeed()
{
    // default: try to connect to /dev/videoX
    _capture = new cv::VideoCapture(_inputSelector);
    if (_capture->isOpened())
    {
        setMode(videoInputMode::USB);
    }
    else
    {
        delete (_capture);
        _capture = NULL;
    }

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

void multiCamVideoFeed::selectInput(int input)
{
    _inputSelector = input;
}

cv::Mat multiCamVideoFeed::getFrame()
{
    cv::Mat result;
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
            (*_capture) >> result;
        }
        break;
      case videoInputMode::MULTICAM:
        _camSysRecv->getCameraFrame(_inputSelector).copyTo(result);
        break;
    }
    return result;
}

