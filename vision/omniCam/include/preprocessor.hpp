// Copyright 2014-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef PREPROCESSOR_HPP
#define PREPROCESSOR_HPP

#include <mutex>
#include <opencv2/opencv.hpp>

#include "configurator.hpp"

class preprocessor {

private:
	cv::Mat full, bgr, bgrExport, hsv;
	configurator *conf;

	int64 start;
	int64 time[20]; // moments the frame has been captured, index 0 is the last one
	double uptime; // calculated time since application running
	double fps; // average FPS over the last number of frames
	int64 count; // amount of frames processed since startup

	std::mutex exportMutex;

public:
	preprocessor(configurator *conf);
	void update(cv::Mat inputImage);
	void updateLocalizationFps(); // call this one for every localization
	double getFps() {
		return fps;
	}
	double getUptime() {
		return uptime;
	}
	int64 getCount() {
		return count;
	}
	std::vector<cv::Mat> getHSVPlanes(void);
	cv::Mat getBgr();
	cv::Mat getHsv();
};

#endif
