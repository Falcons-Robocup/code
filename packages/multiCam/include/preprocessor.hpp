// Copyright 2018 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2014-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef PREPROCESSOR_HPP
#define PREPROCESSOR_HPP

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "configurator.hpp"

class preprocessor
{

private:
	configurator *conf;

	int64 start;
	int64 time[20]; // moments the frame has been captured, index 0 is the last one
	double uptime; // calculated time since application running
	double fps; // average FPS over the last number of frames
	int64 count; // amount of frames processed since startup

public:
	preprocessor(configurator *conf);
	void update();
	double getFps() { return fps; }
	double getUptime() { return uptime; }
	int64 getCount() { return count; }
};

#endif
