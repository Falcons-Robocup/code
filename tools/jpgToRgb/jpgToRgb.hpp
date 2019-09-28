// Copyright 2016-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef JPG_TO_RGB_HPP
#define JPG_TO_RGB_HPP

#include <strings.h>

class jpgToRgb {
private:
    cv::Mat inputFrame;
public:
    jpgToRgb(std::string inputFileName, std::string outputFileName, bool verbose);
    void display();
};

#endif
