// Copyright 2016-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef BALLPOSSESSION_HPP
#define BALLPOSSESSION_HPP

#include <opencv2/opencv.hpp>
#include <mutex>

#include "configurator.hpp"
#include "preprocessor.hpp"

class ballPossession {

private:
	preprocessor *prep;
	configurator *conf;

	cv::Mat hsvFrame;
	cv::Mat inRangeFrame;
	size_t possessionPixels; // size of ball pixels in the possessionFrame window

	std::mutex exportMutex;
	bool busy;

public:
	ballPossession(configurator *conf, preprocessor *prep);
	void update();

	cv::Mat getPossessionHsv();
	cv::Mat getPossessionInRangeFrame();
	size_t getPossessionPixels() {
		return possessionPixels;
	}  // size of ball pixels when ball is near ball handler

	bool getBusy() {
		return busy;
	}
	void setBusy() {
		busy = true;
	}
};

#endif
