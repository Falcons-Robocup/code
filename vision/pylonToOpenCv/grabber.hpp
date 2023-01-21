// Copyright 2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef GRABBER_HPP
#define GRABBER_HPP

#include <mutex>

#include <opencv2/opencv.hpp>
#include <pylon/PylonIncludes.h>

#include "config.hpp"

class grabber {

public:
	grabber();
	~grabber();

	void attach(int index);
	void configure();
	cv::Mat getImage();
	std::string getSerial();
	void start();
	void update();

private:
	void downscale_1_1_2(const uint8_t *buff);

	cv::Mat bgrImage, exportImage;
	Pylon::CInstantCamera *camera;
	std::mutex exportMutex;
	size_t frameCounter;
	Pylon::CGrabResultPtr ptrGrabResult; // smart pointer that will receive the grab result data
	std::string serial;
	bool verbose;
};

#endif // GRABBER_HPP
