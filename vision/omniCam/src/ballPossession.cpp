// Copyright 2016-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "ballPossession.hpp"

using namespace std;
using namespace cv;

ballPossession::ballPossession(configurator *conf, preprocessor *prep) {
	this->conf = conf;
	this->prep = prep;

	exportMutex.lock();
	hsvFrame = Mat::zeros(conf->getPossession().height, conf->getPossession().width, CV_8UC3);
	inRangeFrame = Mat::zeros(conf->getPossession().height, conf->getPossession().width, CV_8UC1);
	exportMutex.unlock();
	possessionPixels = 0;
	busy = false;
}

void ballPossession::update() {
	// determine region of interest (just before the shooter)
	Point center = conf->getCenterPoint();
	size_t width = conf->getPossession().width;
	size_t height = conf->getPossession().height;
	size_t x = center.x - width / 2;
	size_t y = center.y - height - conf->getPossession().yOffset;

	// prepare the color filter
	Scalar possessionMin(conf->getPossession().hue.min, conf->getPossession().sat.min, conf->getPossession().val.min);
	Scalar possessionMax(conf->getPossession().hue.max, conf->getPossession().sat.max, conf->getPossession().val.max);

	exportMutex.lock();
	// get the region of interest
	prep->getHsv()(Rect(x, y, width, height)).copyTo(hsvFrame);
	// append color filter on region of interest
	inRange(hsvFrame, possessionMin, possessionMax, inRangeFrame); // filter ball out
	exportMutex.unlock();

	possessionPixels = countNonZero(inRangeFrame);

	busy = false;
}

cv::Mat ballPossession::getPossessionHsv() {
	exportMutex.lock();
	cv::Mat retVal = hsvFrame.clone();
	exportMutex.unlock();
	return retVal;
}
cv::Mat ballPossession::getPossessionInRangeFrame() {
	exportMutex.lock();
	cv::Mat retVal = inRangeFrame.clone();
	exportMutex.unlock();
	return retVal;
}
