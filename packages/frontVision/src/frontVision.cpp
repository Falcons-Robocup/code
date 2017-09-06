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

// Use the front camera to be able to detect balls:
// - at a higher position (the Parabolic camera only looks down)
// - further away (> 3 meter)
// - higher frame rate


// TODO:
// check calculated (angle, radius) locations by drawing them in viewer (next to x, y determined objects)
// set width and height through v4l2 instead of opencv
// calibrate the radiusAngle

#include <cstdlib>
#include <cerrno>
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <stdio.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "frontVision.hpp"

using namespace std;
using namespace cv;

frontVision::frontVision( ) {
	pause = false;
	viewerUpdateCounter = 0;

	status.fps = 0.0;
	status.frameCounter = 0;
	status.tickCount = 0;
	status.tickCountStart = getTickCount();
	status.uptime = 0.0;
	statusPrevious = status;

	// create once the color image that is later used to copy the ball contour in the round viewer
	ballsContourColorFrame = Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
	ballsContourColorFrame = Scalar(0,255,0); // red

	erodeVal = getStructuringElement(MORPH_RECT, Size(2 * 2 + 1, 2 * 2 + 1), Point(2, 2));
	dilateVal = getStructuringElement(cv::MORPH_RECT, Size(8 * 2 + 1, 8 * 2 + 1), Point(8, 8));

	ballsContourFrame = Mat::zeros( HEIGHT, WIDTH, CV_8UC1 ); // initialize in case the ballSearch is disabled

	capture.open(conf.getVideoDevice());
	if( ! capture.isOpened() ) {
		fprintf( stderr, "ERROR   : %s /dev/video%d\n", strerror(errno), conf.getVideoDevice() );
		exit( EXIT_FAILURE );
	}

	capture.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);

	// calculate once the maximal radiusAngle form the center to one of the four corners
	maxRadiusAngle = sqrt( pow(ANGLE_WIDTH_HALF, 2) + pow(ANGLE_WIDTH_HALF*HEIGHT/WIDTH, 2) );
}

// Filter the ball pixels from the bgr frame
// The ballDilateFrame holds the filtered ball color
void frontVision::ballFilter( ) {
	// convert bgr to hsv
	cvtColor(bgrFrame, hsvFrame, COLOR_BGR2HSV); // convert to HSV

	// filter the ball colors from the bgr frame
	// hue, sat, val
	objectFilterSt ball = conf.getBallFilter( );
	int ballHueMin = ball.hueCenter - (ball.hueDelta-1)/2; // -1 for slider granularity
	int ballHueMax = ball.hueCenter + ball.hueDelta/2;

	Scalar ballMin(ballHueMin, ball.satMin, ball.valMin);
	Scalar ballMax(ballHueMax, ball.satMax, ball.valMax);
	inRange(hsvFrame, ballMin, ballMax, ballFrame); // filter ball out
	erode(ballFrame, ballErodeFrame, erodeVal);
	dilate(ballErodeFrame, ballDilateFrame, dilateVal);
}

// Search for balls in the ballDilateFrame frame
// creates a list of ball objects of type objectSt (x, y, width, height and size)
void frontVision::ballSearch( bool viewer ) {
	contourFrame = ballDilateFrame.clone(); // findCountours function modifies input frame !
	findContours(contourFrame, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // retrieves external contours

	objects.clear(); // start with empty list
	if( viewer ) {
		ballsContourFrame = Mat::zeros( HEIGHT, WIDTH, CV_8UC1 ); // clear contours previous frame
	}

	for (uint ii = 0; ii < contours.size(); ii++)
	{
		// make a bounding rectangle around each contour to calculate the angle
		cv::Rect objectBoundingRectangle = boundingRect( contours.at(ii) );

 		// count the amount of ball pixels in the rectangular area around each contour
		Mat objectBoxFrame = ballFrame(Rect(objectBoundingRectangle.tl(),objectBoundingRectangle.br())); // get the region of interest
		int size = countNonZero( objectBoxFrame );

		// only balls of certain size are used
		if( size > conf.getBallFilter().minimalSize ) {

			// make new ball to gather all information
			objectSt object;
			// normalize the size so 1.0 is a ball that fits the full image
			object.size = 1.0 * size / ( WIDTH * HEIGHT );
			if( object.size > 1.0 ) {
				fprintf( stderr, "ERROR   : calculation error because ball %.3f can not be larger then camera view\n", object.size );
			}

			object.xx = round( objectBoundingRectangle.x + 0.5 * objectBoundingRectangle.width );
	 		object.yy = round( objectBoundingRectangle.y + 0.5 * objectBoundingRectangle.height );

	 		// angle and radius are calculated from center of image instead of upper left corner
	 		double xxCenter = object.xx - WIDTH/2.0;
	 		double yyCenter = object.yy - HEIGHT/2.0;

			// the radius distance from the center to ball in pixels
			double radius = sqrt( pow(xxCenter, 2) + pow(yyCenter, 2) );

			// convert radius to radiusAngle where the center of the screen is 0 degrees
	 		// the following assumptions are made:
	 		//  - the horizontal camera angle is 120 degrees
	 		//  - the pixels are square -> a horizontal pixel has the same angle a a vertical pixel
	 		//  - the pixel to angle conversion is linear, TODO: improve model
	 		// this leads to the following:
	 		//  - a ball straight to the robot has a radiusAngle of 0 degrees
	 		//  - a ball on the left side or right of the x-axis has a radiusAngle of +60 degrees (+ for as well left as right)
	 		//  - a ball halfway on the x-axis has a radiusAngle of +30 degrees, TODO: improve model
	 		//  - the vertical angle is 120*720/1280 = 67.5 degrees
	 		//  - a ball on the top or bottom of the y-axis has a radiusAngle of +34 degrees(+ for as well top as bottom)

	 		// convert the radius in pixels to the radius in angle: radiusAngle = half cameraAngle * pixels/(0.5*WIDTH);
			object.radiusAngle = ANGLE_WIDTH_HALF * ( radius/(0.5*WIDTH) );

			if( object.radiusAngle > maxRadiusAngle ) {
				fprintf( stderr, "ERROR   : calculation error because ball %.2f can not be outside camera view %.2f\n", object.radiusAngle, maxRadiusAngle );
			}

	 		// left of upper y axis is positive angle (y axis is shooter location on robot)
			object.angle = atan2(-xxCenter,-yyCenter); // higher yy value is lower in image

			objects.push_back(object);

			if( viewer ) {
				// for visualization also create an image that shows the contours
				drawContours( ballsContourFrame, contours, ii, 255, 1, 8, hierarchy, 0, Point() );
			}
		}
	}

	// sort the balls from large to small
	sort(objects.begin(), objects.end());
}

void frontVision::viewerStatistics( ) {
	Scalar textColor(0,255,255); // yellow
	char buff[256];
	std::stringstream strForm;
	int printLine = 20;

	strForm << "frame " << setw(6) << status.frameCounter;
	strForm << " uptime " << setw(6) << fixed << setprecision(1) << status.uptime;
	strForm << " fps " << setw(4) << fixed << setprecision(2)  << status.fps;
	putText(viewerFrame, strForm.str(), Point(10,printLine), 1, 1, textColor, 1); printLine+=20; strForm.str("");

 	for( size_t ii = 0; ii < objects.size(); ii++ ) {
 		// put the location of the text right below the center of the object
 		size_t xx = (size_t)objects[ii].xx;
 		if( xx > WIDTH-175 ) { xx = WIDTH-175; }
 		size_t yy = (size_t)objects[ii].yy;
 		if( yy > HEIGHT-20 ) { yy = HEIGHT-20; }
 		Point textPosition = Point(xx,yy)  + Point(20, 10);

 		strForm.str(""); // start with empty buffer
 		strForm << setw(4) << setprecision(1) << objects[ii].angle * 360.0/(2.0*CV_PI); // in viewer angle in degrees

 		int angle = (int)roundf( objects[ii].angle * 360.0/(2.0*CV_PI) ); // for easier interpretation use 360 degrees angle in viewer
 		int radius = (int)roundf( objects[ii].radiusAngle * 360.0/(2.0*CV_PI) ); // for easier interpretation use 360 degrees radiusAngle in viewer
 		int size = (int)roundf( objects[ii].size * WIDTH * HEIGHT ); // for easier interpretation use pixels size in viewer
 		sprintf(buff, "%d %d %d", angle, radius, size );
 		putText(viewerFrame, buff, textPosition, 1, 1, textColor, 1);
 	}
}

bool frontVision::process( ) {

	// the viewer is updated once every n frames
	// the viewerUpdateCounter counter update is also used to update the status
	bool viewerUpdate = false;
	if( viewerUpdateCounter > 9 ) {
		viewerUpdate = true;

		// update the status
		status.tickCount = getTickCount(); // get current time
		status.fps = 1.0 * ( status.frameCounter - statusPrevious.frameCounter ) * getTickFrequency() / ( status.tickCount - statusPrevious.tickCount );
		status.uptime = 1.0 * ( status.tickCount - status.tickCountStart ) / getTickFrequency();

		// update for the next status update cycle
		statusPrevious = status;

		viewerUpdateCounter = 0;
	} else {
		viewerUpdateCounter++;
	}

	capture >> bgrFrame; // get a frame from the camera or file, re-read every cycle

	conf.update( ); // update the values inside the configurator

	ballFilter();
	ballSearch( viewerUpdate );

	if( viewerUpdate && ! pause ) {
		viewerFrame = bgrFrame;
		ballsContourColorFrame.copyTo(viewerFrame, ballsContourFrame != 0); // draw the ball contours in the bgrFrame

		Scalar centerColor(255,0,0); // blue
		Scalar redColor(0,0,255); // red
		size_t bgrLine = WIDTH/40;

	 	for( size_t ii = 0; ii < objects.size(); ii++ ) {
	 		// calculate the center of the object
	 		Point objectCenter = Point(objects[ii].xx, objects[ii].yy );

		 	// draw cross in center of the object
	 		line(viewerFrame, objectCenter - Point(0,bgrLine), objectCenter + Point(0,bgrLine), redColor, 1);
	 		line(viewerFrame, objectCenter - Point(bgrLine,0), objectCenter + Point(bgrLine,0), redColor, 1);

	 		// draw circle for each object where the size indicates the amount of pixels
	 		circle(viewerFrame, objectCenter, 0.3*WIDTH*sqrt(objects[ii].size), redColor, 1);

	 	}

	 	// draw a cross in the center of the viewer
	 	Point viewerCenter = Point(WIDTH/2,HEIGHT/2);
 		line(viewerFrame, viewerCenter - Point(0,bgrLine), viewerCenter + Point(0,bgrLine), centerColor, 1);
 		line(viewerFrame, viewerCenter - Point(bgrLine,0), viewerCenter + Point(bgrLine,0), centerColor, 1);
 		circle(viewerFrame, viewerCenter, 20, centerColor, 1);

 		// if needed resize the viewer
		viewerStatistics( );
		imshow("camera", viewerFrame);

        // write frame to disk very low-frequently, which is useful for calibration
        // this avoids the need for streaming via e.g. teamViewer, which would fill the wifi
        // now one can simply ogg 
        if (status.frameCounter % 20 == 0)
        {
            char filename[90] = {0};
            std::vector<int> jpgCompressionParams;
            jpgCompressionParams.push_back(CV_IMWRITE_JPEG_QUALITY);
            jpgCompressionParams.push_back(40);
	        sprintf(filename, "/var/tmp/frontVisionFrame.jpg");
	        imwrite(filename, viewerFrame, jpgCompressionParams);
        }
		
	}

	status.frameCounter++;

	char key = waitKey(1); //delay N millis, usually long enough to display and capture input
	if( key == 27 || key == 'q' ) { // escape or q
		printf("received escape or q, perform a clean shutdown\n");
		return false;
	} else if ( key == ' ' ) {
		pause = ! pause;
	}
	return true;
}
