// Copyright 2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef RGB_TO_JPG_HPP
#define RGB_TO_JPG_HPP

#include <strings.h>

#include <opencv2/core/core.hpp>

class rgbToJpg {
private:
	bool verbose;
	cv::Mat inputFrame, outputFrame;
	std::string inputFileName, outputFileName;
public:
	rgbToJpg(std::string inputFileName, std::string outputFileName, bool verbose);
	void update();
	void display();
};

#endif
