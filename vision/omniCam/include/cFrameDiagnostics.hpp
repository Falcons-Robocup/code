// Copyright 2016-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef CFRAMEDIAGNOSTICS_HPP
#define CFRAMEDIAGNOSTICS_HPP

#include <iostream>

#include "configurator.hpp"
#include "robotFloor.hpp"
#include "viewer.hpp"

class cFrameDiagnostics {
private:
	// pointers for access to other classes
	configurator *conf;
	robotFloor *rFloor;
	viewer *view;

	bool busy;
	cv::Mat roundFrame;
	cv::Mat floorFrame;

	int fileNameCounter;
	double period; // time between sending two images through diagnostics
	int64 nextTick; // time when next image is send through diagnostics

	char filename[200];
	std::vector<int> jpgCompressionParams;
	std::vector<int> pngCompresionParams;

public:
	cFrameDiagnostics(configurator *conf, robotFloor *rFloor, viewer *view);
	void update();
	void updateAlone();
	bool getBusy() {
		return busy;
	}
	void setBusy() {
		busy = true;
	}
};

#endif
