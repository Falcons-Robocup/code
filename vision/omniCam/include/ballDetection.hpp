// Copyright 2014-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef BALLDETECTION_HPP
#define BALLDETECTION_HPP

#include <opencv2/opencv.hpp>
#include <mutex>

#include "configurator.hpp"
#include "preprocessor.hpp"

struct ballSt {
	int size;
	double radiusTan;
	double radiusLin;
	double angle;
	bool operator<(const ballSt &val) const {
		// sorting this structure is performed on size (higher is better)
		return size > val.size;
	}
};

class ballDetection {

private:
	preprocessor *prep;
	configurator *conf;
	size_t type; // ball detection re-used for cyuan and meganta detection, type is used to distinguish

	cv::Mat contoursFrame;
	cv::Mat dilateFrame;
	cv::Mat erodeFrame;
	cv::Mat inRangeFrame; // used to search the balls
	cv::Mat viewerContoursFrame; // display the balls in the viewer

	std::vector<ballSt> positions;

	std::mutex exportMutex;

	bool busy;

public:

	ballDetection(configurator *conf, preprocessor *prep, size_t type);
	void update(bool viewer);

	std::vector<ballSt> getPositions();
	std::vector<ballSt> getPositionsExport();
	cv::Mat getContoursFrame();
	cv::Mat getDilateFrame();
	cv::Mat getErodeFrame();
	cv::Mat getInRangeFrame();
	cv::Mat getViewerContoursFrame();

	bool getBusy() {
		return busy;
	}
	void setBusy() {
		busy = true;
	}
};

#endif
