// Copyright 2018-2022 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2016-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef NOROS
#include "tracing.hpp"
#endif

#include "localization.hpp"

using namespace std;
using namespace cv;

localization::localization(configurator *conf, determinePosition *detPos, linePointDetection *linePoint,
		preprocessor *prep) {
	this->conf = conf;
	this->detPos = detPos;
	this->linePoint = linePoint;
	this->prep = prep;

	int timeSize = sizeof(time) / sizeof(time[0]);
	for( int ii = 0; ii < timeSize; ii++ ) {
		time[ii] = 0; // fps will be presented as 0 until array is filled with data
	}

	busy = false;
	count = 0;
	fps = 0.0;
}

void localization::update() {
#ifndef NOROS
    TRACE_FUNCTION("");
#endif
	linePoint->update();

	detPos->pointsToPosition();

	// keep track of the last number of times when the localization was performed to be able to calculate the average localization FPS
	int timeSize = sizeof(time) / sizeof(time[0]);
	for( int ii = timeSize - 1; ii > 0; ii-- ) {
		time[ii] = time[ii - 1];
	}
	time[0] = getTickCount();

	// calculate FPS
	fps = (timeSize - 1) * 1.0 / ((time[0] - time[timeSize - 1]) / getTickFrequency());

	// keep track how many frames have been processed
	count++;

	// inform higher level the thread has been finished
	busy = false;
}
