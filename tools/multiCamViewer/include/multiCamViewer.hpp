// Copyright 2016-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef MULTI_CAM_VIEWER_HPP
#define MULTI_CAM_VIEWER_HPP

#include "camSysReceive.hpp"

#include <thread>

class multiCamViewer {
private:
    camSysReceive *camSysRecv;
	std::thread camSysRecvThread;

	int singleCam;

public:
	multiCamViewer(int singleCam);
	~multiCamViewer();
	bool update();
};

#endif
