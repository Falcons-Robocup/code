/*
 * multiCamVideoFeed.hpp
 * Adapter layer on top of multiCam data producer (cameraReceive.hpp).
 * Can also connect to /dev/video (handy for testing with a webcam) or set a still image.
 * Intended for, but not limited to, use in optiCal.
 *
 *  Created on: May, 2018
 *      Author: Jan Feitsma
 */

#ifndef MULTICAMVIDEOFEED_HPP_
#define MULTICAMVIDEOFEED_HPP_

#include <string>

#include <opencv2/opencv.hpp>
#include <thread>


#include "camSysReceive.hpp"
#include "grabber.hpp"

enum class videoInputMode
{
    NONE,
    STILL,       // a png or jpg image
    USB,         // /dev/videoX
    MULTICAM,    // Andre's code, default camera 0, can select also cameras 1,2,3
    PYLONCAM     // Basler Pylon Dart USB camera's, can select also camera 1,2 and 3
};


class multiCamVideoFeed
{
  public:
    multiCamVideoFeed();
    ~multiCamVideoFeed();
    void setMode(videoInputMode const &mode);
    void setStill(std::string const &filename);
    void setUsb(int device);
    void setPylon();
    void selectInput(int input);
    cv::Mat getFrame();
    std::string describe();

  private:
    videoInputMode    _mode = videoInputMode::MULTICAM;
    camSysReceive    *_camSysRecv;
    std::thread       _camSysRecvThread;
    cv::VideoCapture *_capture = NULL;
    int               _inputSelector = 0;
    std::string       _overrideFile;
    cv::Mat           _overrideFrame;
    grabber          *_pylonGrab[4];
    std::thread       _pylonGrabThread[4];
};

#endif /* MULTICAMVIDEOFEED_HPP_ */

