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

#ifndef FRONT_VISION_HPP
#define FRONT_VISION_HPP

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "types/frontVisionTypes.hpp"
#include "configurator.hpp"

// about 146% on HP Elitebook with only 22 fps
// #define WIDTH 1920
// #define HEIGHT 1080

// about 87% on HP Elitebook
#define WIDTH 1280
#define HEIGHT 720

// about 46% on HP Elitebook
// this is not a wide resultion
// #define WIDTH 800
// #define HEIGHT 600

// about 28% on HP Elitebook
// this is not a wide resultion
// #define WIDTH 640
// #define HEIGHT 480

// about 6% on HP Elitebook
// this is not a wide resultion
// #define WIDTH 320
// #define HEIGHT 240

// the camera angle is 120 degrees in the horizontal plane
#define ANGLE_WIDTH_HALF ( 2.0*CV_PI*60.0/360.0 )

class frontVision
{
private:

	bool pause; // toggle the viewer between pause and update
	size_t viewerUpdateCounter; // the viewer is not updated every frame
	double maxRadiusAngle; // used for out of range checking

	cv::VideoCapture capture;
	cv::Mat bgrFrame, hsvFrame, ballFrame, ballErodeFrame, ballDilateFrame;
	cv::Mat contourFrame, ballsContourFrame, ballsContourColorFrame, viewerFrame;

	statusSt status, statusPrevious; // status information available for higher level e.g. ros

	std::vector<objectSt> objects; // vector to store the basic properties of balls found in one frame

	configurator conf; // contains the filter and other configuration settings

	// the following matrices are used for the ball filtering
	cv::Mat erodeVal; // used for the ball erode function
	cv::Mat dilateVal; // used for the ball dilate function

	// the following vectors are used in the ball search function
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	void ballFilter( );
	void ballSearch( bool viewer );
	void showTrackBars( );
	void viewerStatistics( );

public:

	frontVision( );
	~frontVision( ) { capture.release(); }

	// capture and process input frame
	// returns true until abort requested
	bool process( );

	// list of balls found after process
	// these need to be exported to the ros interface
	std::vector<objectSt> getBalls( ) { return objects; }

	// provide status information for higher levels e.g. ros
	statusSt getStatus( ) { return status; }
};

#endif
