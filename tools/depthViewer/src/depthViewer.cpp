// Copyright 2015 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

// basic kinect2 viewer
// displays the rgb image, ir image, depth image and "registered" image
// based on https://github.com/OpenKinect/libfreenect2/blob/master/examples/Protonect.cpp

#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <cstdlib>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace libfreenect2;
using namespace cv;

bool depthViewerShutDown = false; // global variable used to stop the main loop when cntr-c signal

// #define SHOW_REGISTERED

struct ballSt {
	int size;
	size_t xx;
	size_t width;
	size_t yy;
	size_t height;
	bool operator<( const ballSt& val ) const {
		// sorting this struct is performed on size (higher is better)
		return size > val.size;
	}
};

Mat_<Vec3b> bgrFrame(1080, 1920);
Mat hsv, ballFrame, ballFrameSmall, ballErodeFrame, ballDilateFrame, ballDilateFrameSmall, contourFrame, ballsContourFrame, ballsContourFrameSmall, ballsContourColorFrame;
size_t ballPixels;
vector<ballSt> positions;
Vec3b ballDrawColor(0,255,255); // yellow
int centerX = 105;
int centerY = 116;
int linX = 128;
int linY = 88;
int ballHueCenter = 22;
int ballHueDelta = 9;
int ballSatMin = 125;
int ballSatMax = 255;
int ballValMin = 110;
int ballValMax = 255;

// interrupter signal received e.g. ctrl-c
void sigIntHandler(int s) {
	printf("received interrupt signal, probably ctrl-c, do a clean shutdown\n");
	depthViewerShutDown = true;
}

// Filter the ball pixels from the bgr frame
// The ballDilateFrame holds the filtered ball color
void ballFilter( ) {
	// convert bgr to hsv
	cvtColor(bgrFrame, hsv, COLOR_BGR2HSV); // convert to HSV

	// filter the ball colors from the bgr frame
	// hue, sat, val

	int ballHueMin = ballHueCenter - (ballHueDelta-1)/2; // -1 for slider granularity
	int ballHueMax = ballHueCenter + ballHueDelta/2;

	Scalar ballMin(ballHueMin, ballSatMin, ballValMin);
	Scalar ballMax(ballHueMax, ballSatMax, ballValMax);
	inRange(hsv, ballMin, ballMax, ballFrame); // filter ball out
	Mat erodeVal = getStructuringElement(MORPH_RECT, Size(2 * 2 + 1, 2 * 2 + 1), Point(2, 2));
	erode(ballFrame, ballErodeFrame, erodeVal);
	Mat dilateVal = getStructuringElement(cv::MORPH_RECT, Size(8 * 2 + 1, 8 * 2 + 1), Point(8, 8));
	dilate(ballErodeFrame, ballDilateFrame, dilateVal);
}

// Search for balls in the ballDilateFrame frame
// creates a list of ball positions of type ballSt (x,y,width,height and size)
void ballSearch( )
{
	Rect objectBoundingRectangle;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	contourFrame = ballDilateFrame.clone(); // findCountours function modifies input frame !
	findContours(contourFrame, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // retrieves external contours

	positions.clear(); // start with empty list
	ballsContourFrame = Mat::zeros( 1080, 1920, CV_8UC1 ); // clear contours previous frame

	for (uint ii = 0; ii < contours.size(); ii++)
	{
		// make new ball to gather all information
		ballSt ball;

		// make a bounding rectangle around each contour to calculate the angle
		objectBoundingRectangle = boundingRect( contours.at(ii) );

		ball.xx = objectBoundingRectangle.x;
		ball.width = objectBoundingRectangle.width;
		ball.yy = objectBoundingRectangle.y;
		ball.height = objectBoundingRectangle.height;

		// count the amount of ball pixels in the rectangular area around each contour
		Mat ballBoxFrame = ballFrame(Rect(objectBoundingRectangle.tl(),objectBoundingRectangle.br())); // get the region of interest
		ball.size = countNonZero( ballBoxFrame );

		// only balls of certain size are used
		if( ball.size > 200 ) {
			drawContours( ballsContourFrame, contours, ii, 255, 2, 8, hierarchy, 0, Point() ); // thicker line because of viewer is scaled down
			positions.push_back(ball);
		}
	}
}


// very basic function to translate pixels from bgr frame to ir frame
// use center of both fields to align
// center of bgr frame = center of ir frame
// the bgr (1920x1080) has 16:9 ratio, while the ir (512x424) has more a more 4:3 ratio
// also they do not have exactly the same view area
// Note: the camera's have an offset in the x direction that results in a large error when nearby !
Point bgrToDepth( Point bgrPoint ) {
	Point depthPoint;
	float centeredX = bgrPoint.x - 1919.0/2.0 + centerX - 100; // 0 means on Y axis (even pixels => no exact middle line)
	float centeredY = bgrPoint.y - 1079.0/2.0 + centerY - 100; // 0 means on X axis (even lines => no exact middle line)
	float xConversion = linX * 512.0/(1920.0*100.0); // TODO add border difference
	float yConversion = linY * 424.0/(1080.0*100.0); // TODO add border difference
	depthPoint.x = round( centeredX * xConversion + 511.0/2.0 );
	depthPoint.y = round( centeredY * yConversion + 423.0/2.0 );
	return depthPoint;
}

void showTrackBars( ) {
	std::string calWindow = "calibration";
	namedWindow(calWindow, CV_WINDOW_NORMAL);
	createTrackbar( "Center X", calWindow, &centerX, 200 );
	createTrackbar( "Center Y", calWindow, &centerY, 200 );
	createTrackbar( "Linear X", calWindow, &linX, 200 );
	createTrackbar( "Linear Y", calWindow, &linY, 200 );
	createTrackbar( "Hue Cent", calWindow, &ballHueCenter, 179); // hue range is 0 to 179
	createTrackbar( "Hue Delta", calWindow, &ballHueDelta, 180);
	createTrackbar( "Sat Min", calWindow, &ballSatMin, 255);
	createTrackbar( "Sat Max", calWindow, &ballSatMax, 255);
	createTrackbar( "Val Min", calWindow, &ballValMin, 255);
	createTrackbar( "Val Max", calWindow, &ballValMax, 255);
}

int main(int argc, char *argv[]) {

	bool updateViewer = true;
	bool sqrtUse = true;
	Mat_<uchar> irFrame(424, 512);
	Mat_<unsigned short> depthFrame(424, 512);
	Mat_<Vec3b> regiFrame(424, 512);
	Mat depthFrameScale, bgrFrameSmall; // used by the viewer

	// create once the color image that is later used to copy the ball contour in the round viewer
	ballsContourColorFrame = Mat::zeros(1080, 1920, CV_8UC3);
	ballsContourColorFrame = Scalar(0,255,0); // red

	Freenect2 freenect2;
	if( freenect2.enumerateDevices() == 0 ) {
		printf("no device connected\n");
		exit(EXIT_FAILURE);
	}

	string serial = freenect2.getDefaultDeviceSerialNumber();

	Freenect2Device *dev = freenect2.openDevice(serial);

	// redirect cntr-c signal to do a clean shutdown
	signal(SIGINT, sigIntHandler);

	// create listener
	SyncMultiFrameListener listener( Frame::Color | Frame::Ir	| Frame::Depth);

	dev->setColorFrameListener(&listener);
	dev->setIrAndDepthFrameListener(&listener);

	dev->start();
	// now the serial and firmware version are available
	printf( "device serial number: %s\n", dev->getSerialNumber().c_str());
	printf( "device firmware version: %s\n", dev->getFirmwareVersion().c_str());

	FrameMap frames;

#ifdef SHOW_REGISTERED
	Registration* registration = new Registration( dev->getIrCameraParams(), dev->getColorCameraParams() );
	Frame undistorted(512, 424, 4), registered(512, 424, 4);
	// create a pointer from the above created registered to be used later on in the function
	Frame *regi = &registered;
#endif

	size_t verboseCounter = 0;

	while( ! depthViewerShutDown ) { // stop when cntr-c
		listener.waitForNewFrame(frames);
		// collect the rgb, ir and depth frame
		Frame *rgb = frames[Frame::Color]; // 1920x1080 32-bit BGRX.
		Frame *ir = frames[Frame::Ir]; // 512x424 float. Range is [0.0, 65535.0].
		Frame *depth = frames[Frame::Depth]; // 512x424 float, unit: millimeter. Non-positive, NaN, and infinity are invalid or missing data.
#ifdef SHOW_REGISTERED
		// correlate the color camera with the depth camera
		registration->apply(rgb, depth, &undistorted, regi);
#endif
		if( verboseCounter == 0 ) { // only the first time
			verboseCounter = 100;
			printf( "rgb   width %4d, height %4d, bytes per pixel %d\n", (int)rgb->width, (int)rgb->height, (int)rgb->bytes_per_pixel );
			printf( "ir    width %4d, height %4d, bytes per pixel %d\n", (int)ir->width, (int)ir->height, (int)ir->bytes_per_pixel);
			printf( "depth width %4d, height %4d, bytes per pixel %d\n", (int)depth->width, (int)depth->height, (int)depth->bytes_per_pixel);
		} else if( verboseCounter == 1 ) { // verbose once in a while
			verboseCounter = 200;
			printf( "rgb   exposure %2.1f, gain %1.2f, gamma %1.2f\n", rgb->exposure, rgb->gain, rgb->gamma );

		} else {
			verboseCounter--;
		}

		showTrackBars(); // if needed update the conversion factors

		// convert bgr frame from the libfreenect2 format to openCV matrix format
		// also the mirror the image (so it is what you would expect)
		for( size_t yy = 0; yy < 1080; yy++ ) { // lines
			for( size_t xx = 0; xx < 1920; xx++ ){ // pixels on line
				size_t pixel =  ( xx + 1920 * yy ) * rgb->bytes_per_pixel;
				// pixel byte 0 = blue, byte 1 = green, byte 2 = red, byte 3 is not used (and is always 255)
			    bgrFrame(yy, 1919-xx) = Vec3b( rgb->data[pixel + 0], rgb->data[pixel + 1], rgb->data[pixel + 2] );
			}
		}
		// convert ir frame from the libfreenect2 format to openCV matrix format
		// also the mirror the image (so it is what you would expect)
		for( size_t yy = 0; yy < 424; yy++ ) { // lines
			for( size_t xx = 0; xx < 512; xx++ ){ // pixels on line
				size_t pixel =  ( xx + 512 * yy ) * ir->bytes_per_pixel;
				float *floatValue = (float *) &ir->data[pixel]; // convert uchar pointer to float pointer
				if( *floatValue >= 65536.0 ) { printf( "Error ir max value %.0f\n", *floatValue ); }
				if( *floatValue < -0.00001 ) { printf( "Error ir min value %.2f\n", *floatValue ); }
				float value = sqrt(*floatValue);
				if( sqrtUse ) {
					value = 3.0 * sqrt(*floatValue); // "linearize" the 0 to 65k range to 0 to 255 and make it brighter
				} else {
					value = (*floatValue/190.0); // "linearize" the 0 to 65k range to 0 to 255 and make it brighter
				}
				if( value > 255 ) { value = 255; }
				irFrame(yy, 511-xx) = (uchar) value;
			}
		}

		// convert depth frame from the libfreenect2 format to openCV matrix format
		// also the mirror the image (so it is what you would expect)
		for( size_t yy = 0; yy < 424; yy++ ) { // lines
			for( size_t xx = 0; xx < 512; xx++ ){ // pixels on line
				size_t pixel =  ( xx + 512 * yy ) * depth->bytes_per_pixel;
				float *floatValue = (float *) &depth->data[pixel]; // convert uchar pointer to float pointer
				if( *floatValue >= 65536 ) { printf( "Error depth max value %.0f\n", *floatValue ); *floatValue = 65535; } // 65 meters
				if( *floatValue < -0.00001 ) { printf( "Error depth min value %.2f\n", *floatValue ); *floatValue = 0.0; }
			    // depthFrame(yy, xx) = (uchar) (255.0 - *floatValue * 255.0 / maxDistance); // scale 0 to maxDistance to 0-255 (e.g. granularity = 10000/255 = 40mm)
				if( *floatValue == 0.0 ) {
					depthFrame(yy, 511-xx) = 65535;
				} else {
					depthFrame(yy, 511-xx) = (unsigned short) (*floatValue);
				}
			}
		}

#ifdef SHOW_REGISTERED
		// convert registered frame from the libfreenect2 format to openCV matrix format
		// also the mirror the image (so it is what you would expect)
		for( size_t yy = 0; yy < 424; yy++ ) { // lines
			for( size_t xx = 0; xx < 512; xx++ ){ // pixels on line
				size_t pixel =  ( xx + 512 * yy ) * regi->bytes_per_pixel;
				// float *floatValue = (float *) &depth->data[pixel]; // convert uchar pointer to float pointer
				// if( *floatValue >= 65536 ) { printf( "Error depth max value %.0f\n", *floatValue ); *floatValue = 65535; } // 65 meters
				// if( *floatValue < -0.00001 ) { printf( "Error depth min value %.2f\n", *floatValue ); *floatValue = 0.0; }
			    // depthFrame(yy, xx) = (uchar) (255.0 - *floatValue * 255.0 / maxDistance); // scale 0 to maxDistance to 0-255 (e.g. granularity = 10000/255 = 40mm)
				regiFrame(yy, 511-xx) = Vec3b( regi->data[pixel + 0], regi->data[pixel + 1], regi->data[pixel + 2] );
			}
		}
#endif

		ballFilter();
		ballSearch();

		float brightness = 2.0;
		depthFrame.convertTo(depthFrameScale, CV_8UC1, -255.0/10000.0, 255); // for visualization 6 meters is good enough

		if( updateViewer ) {
		 	ballsContourColorFrame.copyTo(bgrFrame, ballsContourFrame != 0); // draw the ball contours in the bgrFrame
			Scalar centerColor(0,0,0); // black
			Scalar textColor(0,255,0); // green
			Scalar redColor(0,0,255); // red
			size_t bgrLine = 30;
	 		size_t depthLine = 10;
	 		stringstream strForm;

		 	for( size_t ii = 0; ii < positions.size(); ii++ ) {

		 		Point bgrCenter;
		 		bgrCenter.x = round( positions[ii].xx + 0.5 * positions[ii].width );
		 		bgrCenter.y = round( positions[ii].yy + 0.5 * positions[ii].height );

		 		Point depthCenter = bgrToDepth(bgrCenter);
		 		unsigned short distance = depthFrame(depthCenter.y, depthCenter.x);
		 		// the mapping from the bgr to depth is not perfectly, checkout also the depths in the nearby area
		 		// and then choose the lowest one, except when it is 0, which probably means no correct measurement
		 		int delta = 10;
		 		unsigned short distanceTemp = depthFrame(depthCenter.y, depthCenter.x + delta);
		 		if( distanceTemp < distance ) { distance = distanceTemp; }
		 		distanceTemp = depthFrame(depthCenter.y, depthCenter.x - delta);
		 		if( distanceTemp < distance ) { distance = distanceTemp; }
		 		distanceTemp = depthFrame(depthCenter.y + delta, depthCenter.x);
		 		if( distanceTemp < distance ) { distance = distanceTemp; }
		 		distanceTemp = depthFrame(depthCenter.y - delta, depthCenter.x);
		 		if( distanceTemp < distance ) { distance = distanceTemp; }

		 		distanceTemp = depthFrame(depthCenter.y, depthCenter.x + 0.25*delta);
		 		if( distanceTemp < distance ) { distance = distanceTemp; }
		 		distanceTemp = depthFrame(depthCenter.y, depthCenter.x - 0.25*delta);
		 		if( distanceTemp < distance ) { distance = distanceTemp; }
		 		distanceTemp = depthFrame(depthCenter.y + 0.25*delta, depthCenter.x);
		 		if( distanceTemp < distance ) { distance = distanceTemp; }
		 		distanceTemp = depthFrame(depthCenter.y - 0.25*delta, depthCenter.x);

		 		distanceTemp = depthFrame(depthCenter.y, depthCenter.x + 0.50*delta);
		 		if( distanceTemp < distance ) { distance = distanceTemp; }
		 		distanceTemp = depthFrame(depthCenter.y, depthCenter.x - 0.50*delta);
		 		if( distanceTemp < distance ) { distance = distanceTemp; }
		 		distanceTemp = depthFrame(depthCenter.y + 0.50*delta, depthCenter.x);
		 		if( distanceTemp < distance ) { distance = distanceTemp; }
		 		distanceTemp = depthFrame(depthCenter.y - 0.50*delta, depthCenter.x);

		 		distanceTemp = depthFrame(depthCenter.y, depthCenter.x + 0.75*delta);
		 		if( distanceTemp < distance ) { distance = distanceTemp; }
		 		distanceTemp = depthFrame(depthCenter.y, depthCenter.x - 0.75*delta);
		 		if( distanceTemp < distance ) { distance = distanceTemp; }
		 		distanceTemp = depthFrame(depthCenter.y + 0.75*delta, depthCenter.x);
		 		if( distanceTemp < distance ) { distance = distanceTemp; }
		 		distanceTemp = depthFrame(depthCenter.y - 0.75*delta, depthCenter.x);

		 		// draw cross in center (that later on displays the distance)
		 		line(bgrFrame, bgrCenter - Point(0,bgrLine), bgrCenter + Point(0,bgrLine), redColor, 2);
		 		line(bgrFrame, bgrCenter - Point(bgrLine,0), bgrCenter + Point(bgrLine,0), redColor, 2);

		 		// draw in depth frame
		 		line(depthFrameScale, depthCenter - Point(0,depthLine), depthCenter + Point(0,depthLine), 255, 1);
		 		line(depthFrameScale, depthCenter - Point(depthLine,0), depthCenter + Point(depthLine,0), 255, 1);

		 		// draw also in ir frame
		 		line(irFrame, depthCenter - Point(0,depthLine), depthCenter + Point(0,depthLine), 255, 1);
		 		line(irFrame, depthCenter - Point(depthLine,0), depthCenter + Point(depthLine,0), 255, 1);

		 		// put text below (which later on needs to be used to show the distance)
		 		size_t leftX = round( positions[ii].xx );
		 		size_t bottomY = round( positions[ii].yy + positions[ii].height + 50 );
		 		strForm.str("");
		 		strForm << distance;
		 		putText(bgrFrame, strForm.str(), Point(leftX,bottomY), 1, 2, textColor, 2); // thicker and larger font because of scaling

		 		Point irText = bgrToDepth(Point(leftX, bottomY));
		 		putText(irFrame, strForm.str(), irText + Point(0, 10), 1, 1, 255, 1); // thicker and larger font because of scaling
		 		putText(depthFrameScale, strForm.str(), irText + Point(0, 10), 1, 1, 255, 1); // thicker and larger font because of scaling
		 	}

		 	// draw a cross in the center of all 3 images
		 	Point rgbCenter = Point(1920/2,1080/2);
		 	Point depthCenter = bgrToDepth( rgbCenter );

	 		line(bgrFrame, rgbCenter - Point(0,bgrLine), rgbCenter + Point(0,bgrLine), centerColor, 2);
	 		line(bgrFrame, rgbCenter - Point(bgrLine,0), rgbCenter + Point(bgrLine,0), centerColor, 2);
	 		circle(bgrFrame, rgbCenter, 20, centerColor, 2);
	 		line(depthFrameScale, depthCenter - Point(0,depthLine), depthCenter + Point(0,depthLine), 255, 1);
	 		line(depthFrameScale, depthCenter - Point(depthLine,0), depthCenter + Point(depthLine,0), 255, 1);
	 		circle(depthFrameScale, depthCenter, 6, 255, 1);
	 		line(irFrame, depthCenter - Point(0,depthLine), depthCenter + Point(0,depthLine), 255, 1);
	 		line(irFrame, depthCenter - Point(depthLine,0), depthCenter + Point(depthLine,0), 255, 1);
	 		circle(irFrame, depthCenter, 6, 255, 1);

			resize(bgrFrame, bgrFrameSmall, Size(rgb->width/2,rgb->height/2));
			resize(ballFrame, ballFrameSmall, Size(rgb->width/3,rgb->height/3));
			resize(ballsContourFrame, ballsContourFrameSmall, Size(rgb->width/3,rgb->height/3));
			resize(ballDilateFrame, ballDilateFrameSmall, Size(rgb->width/3,rgb->height/3));


			imshow("IR, press esc to quit", irFrame);
			imshow("Depth, press esc to quit", depthFrameScale);
#ifdef SHOW_REGISTERED
			imshow("Registered, press esc to quit", regiFrame);
#endif
			// imshow("Ball, press esc to quit", ballFrameSmall);
			// imshow("Balls Contour, press esc to quit", ballsContourFrameSmall);
			// imshow("Balls Dilate, press esc to quit", ballDilateFrameSmall);
			imshow("BGR, press esc to quit", bgrFrameSmall);
		}
   		char key = waitKey(1); //delay N millis, usually long enough to display and capture input
		if( key == 27 || key == 'q' ) { // escape or q
			printf("received escape or q, perform a clean shutdown\n");
			depthViewerShutDown = true;
		} else if( key == 's' ) {
			sqrtUse = ! sqrtUse;
		} else if( key == 'u' ) {
			updateViewer = ! updateViewer;
		}

		// probably to be able to receive the next 3 frames
		listener.release(frames);
	}

	dev->stop();
	dev->close();
#ifdef SHOW_REGISTERED
	delete registration;
#endif
	return 0;
}

