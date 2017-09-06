 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2016 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#include "ballPossession.hpp"

using namespace std;
using namespace cv;

ballPossession::ballPossession(configurator *conf, preprocessor *prep) {
	this->conf = conf;
	this->prep = prep;

	hasPossession = false;
	exportMutex.lock();
	hsvFrame = 	Mat::zeros( conf->getPossession().height, conf->getPossession().width, CV_8UC3 );
	inRangeFrame = Mat::zeros( conf->getPossession().height, conf->getPossession().width, CV_8UC1 );
	exportMutex.unlock();
	possessionPixels = 0;
	busy = false;
}

void ballPossession::update( ) {
	// determine region of interest (just before the shooter)
	Point center = conf->getCenterPoint();
	size_t width = conf->getPossession().width;
	size_t height = conf->getPossession().height;
	size_t x = center.x - width/2;
	size_t y = center.y - height - conf->getPossession().yOffset;

	// prepare the color filter
	Scalar possessionMin(conf->getPossession().hue.min, conf->getPossession().sat.min, conf->getPossession().val.min);
	Scalar possessionMax(conf->getPossession().hue.max, conf->getPossession().sat.max, conf->getPossession().val.max);

	exportMutex.lock();
	// get the region of interest
	prep->getHsv()(Rect(x,y,width,height)).copyTo(hsvFrame);
	// append color filter on region of interest
	inRange(hsvFrame, possessionMin, possessionMax, inRangeFrame); // filter ball out
	exportMutex.unlock();

	possessionPixels = countNonZero( inRangeFrame );

	hasPossession = ( possessionPixels >= conf->getPossession().threshold );

	if( hasPossession )
	{
		rosNotivy(true);
	}
	busy = false;
}

cv::Mat ballPossession::getPossessionHsv( ) {
	exportMutex.lock();
	cv::Mat retVal = hsvFrame.clone();
	exportMutex.unlock();
	return retVal;
}
cv::Mat ballPossession::getPossessionInRangeFrame( ) {
	exportMutex.lock();
	cv::Mat retVal = inRangeFrame.clone();
	exportMutex.unlock();
	return retVal;
}

// TKOV: please check below
void ballPossession::rosNotivy(const bool ballIsSeen ) {
	for (vector<observer*>::const_iterator iter = vecObservers.begin(); iter != vecObservers.end(); iter++)
	{
		if (*iter != NULL)
		{
			(*iter)->update_own_ball_possession(ballIsSeen);
		}
	}
}

void ballPossession::attach(observer *observer)
{
    vecObservers.push_back(observer);
}

void ballPossession::detach(observer *observer)
{
    vecObservers.erase(std::remove(vecObservers.begin(), vecObservers.end(), observer), vecObservers.end());
}


