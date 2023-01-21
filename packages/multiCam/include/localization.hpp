// Copyright 2018-2022 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2016-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

#include <opencv2/opencv.hpp>

#include "configurator.hpp"
#include "determinePosition.hpp"
#include "linePointDetection.hpp"
#include "preprocessor.hpp"

class localization {

private:
	configurator *conf;
	determinePosition *detPos;
	linePointDetection *linePoint;
	preprocessor *prep;

	bool busy;
	int64 time[20]; // moments the localization has been captured, index 0 is the last one
	double fps;
	int64 count; // amount of localizations performed since startup

public:
	localization(configurator *conf, determinePosition *detPos, linePointDetection *linePoint, preprocessor *prep);
	void update();
	double getFps() {
		return fps;
	}
	int64 getCount() {
		return count;
	}
	bool getBusy() {
		return busy;
	}
	void setBusy() {
		busy = true;
	}
};
#endif
