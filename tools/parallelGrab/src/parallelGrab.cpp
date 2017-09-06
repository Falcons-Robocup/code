// Copyright 2015 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

// Simple test application to grab from multiple camera's in parallel to measure the
// camera interrelation


// got 30fps with 5 camera's under the following conditions
// auto mode with enough light (so camera's use short shutter and exposure time)
// 3 c525
// 2 f100
// all on 800x600
// hp elitebook 8570w
// 4 directly connected on laptop
// 1 connected through docking station
// imshow displays 4*600 x 2*400 = 2400x800

// with enabled viewer:
// - 640x480 (yuyv) @ 30fps (system load is 104%)
// - 800x600 (mjpeg) @ 30fps (system load is 160%)
// with disabled viewer:
// - 1280x720 (mjpeg) @ 30 fps (system load 220%)
// - 1920x1080 (mjpeg) @ 20 fps (system load 320%)

// note: found out that camera resolutions of 640x480 and lower use YUYV mode (raw data)
// 800x600 and above MJPEG

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <iostream>
#include <fstream>
#include <cerrno>
#include <fcntl.h>    /* For O_RDWR */

using namespace cv;
using namespace std;

typedef struct
{
   int id;
} handleT;

Mat cam0Frame, cam1Frame, cam2Frame, cam3Frame, cam4Frame, cam5Frame, cam6Frame, cam7Frame;

int widthCam = 800;
int heigthCam = 600;
int leftCol = 100;
int rightCol = widthCam-100;
int widthCol = rightCol - leftCol;
int topRow = 100;
int bottomRow = heigthCam-100;
int heigth = bottomRow - topRow;
Mat combined(2*heigth, 4*widthCol, CV_8UC3);


VideoCapture cam0("0");
VideoCapture cam1("1");
VideoCapture cam2("2");
VideoCapture cam3("3");
VideoCapture cam4("4");
VideoCapture cam5("5");
VideoCapture cam6("6");
VideoCapture cam7("7");

void *processCamera( void *param ) {
	handleT * ip = (handleT *) param;
	int id = ip->id;
	// printf("%2d\n", id);
	switch( id ) {
		case 1:
			cam1 >> cam1Frame;
			cam1Frame.colRange(leftCol, rightCol).rowRange(topRow, bottomRow).copyTo(combined.colRange(1*widthCol, 2*widthCol).rowRange(0*heigth, 1*heigth));
			break;
		case 2:
			cam2 >> cam2Frame;
			cam2Frame.colRange(leftCol, rightCol).rowRange(topRow, bottomRow).copyTo(combined.colRange(2*widthCol, 3*widthCol).rowRange(0*heigth, 1*heigth));
			break;
		case 3:
			cam3 >> cam3Frame;
			cam3Frame.colRange(leftCol, rightCol).rowRange(topRow, bottomRow).copyTo(combined.colRange(3*widthCol, 4*widthCol).rowRange(0*heigth, 1*heigth));
			break;
		case 4:
			cam4 >> cam4Frame;
			cam4Frame.colRange(leftCol, rightCol).rowRange(topRow, bottomRow).copyTo(combined.colRange(0*widthCol, 1*widthCol).rowRange(1*heigth, 2*heigth));
			break;
		case 5:
			cam5 >> cam5Frame;
			cam5Frame.colRange(leftCol, rightCol).rowRange(topRow, bottomRow).copyTo(combined.colRange(1*widthCol, 2*widthCol).rowRange(1*heigth, 2*heigth));
			break;
		case 6:
			cam6 >> cam6Frame;
			cam6Frame.colRange(leftCol, rightCol).rowRange(topRow, bottomRow).copyTo(combined.colRange(2*widthCol, 3*widthCol).rowRange(1*heigth, 2*heigth));
			break;
		case 7:
			cam7 >> cam7Frame;
			cam7Frame.colRange(leftCol, rightCol).rowRange(topRow, bottomRow).copyTo(combined.colRange(3*widthCol, 4*widthCol).rowRange(1*heigth, 2*heigth));
			break;
		default:
			cam0 >> cam0Frame;
			cam0Frame.colRange(leftCol, rightCol).rowRange(topRow, bottomRow).copyTo(combined.colRange(0*widthCol, 1*widthCol).rowRange(0*heigth, 1*heigth));
			break;
	}
	return 0;
}

int main()
{
	char key;
	double previousTime = (double)getTickCount();
	double tickFrequency = getTickFrequency();
	int fpsCounter = 0;

	ifstream cam0File("/dev/video0");
	if( cam0File.good() ) {
		printf( "cam0 uses usb camera\n" );
		cam0.open(0);
		// cam0.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G') ); // does not seem to work for the c525 and f100
		cam0.set(CV_CAP_PROP_FRAME_WIDTH, widthCam);
		cam0.set(CV_CAP_PROP_FRAME_HEIGHT, heigthCam);
		// cam0.set(CV_CAP_PROP_FPS, 20); // works for c525 (up to 30fps), but not for f100

#ifdef NONO
		int fd = open("/dev/video0", O_RDWR);
		struct v4l2_capability caps; // APOX with " = {0}; " results in warnings on ubuntu 12.04 / ros

		// show type of camera
		if (-1 == ioctl(fd, VIDIOC_QUERYCAP, &caps))
		{
			cerr << endl;
			cerr << "Vision: dynamicCalibration Error when querying capabilities" << endl;
			cerr << "Error: " << strerror(errno) << endl;
			exit (EXIT_FAILURE);
		}
		int versionMajor = (int)(caps.version>>16)&&0xff;
		int versionMinor = (int)(caps.version>>24)&&0xff;
		cout << "driver " << caps.driver << ", card " << caps.card << ", bus " << caps.bus_info;
		cout << ", version " << versionMajor << "." << versionMinor;
		cout << ", reserved " <<  caps.reserved;
		cout << ", device caps 0x" << hex << caps.device_caps;
		cout << ", capabilities 0x" << hex << caps.capabilities << dec << endl;
#endif
	} else {
		string fileName = "/home/robocup/data/bld66_jun24_falcon1_2.avi";
		printf( "cam0 uses file: %s\n", fileName.c_str());
		cam0.open(fileName.c_str());
	}

	ifstream cam1File("/dev/video1");
	if( cam1File.good() ) {
		printf( "cam1 uses usb camera\n");
		cam1.open(1);
		cam1.set(CV_CAP_PROP_FRAME_WIDTH, widthCam);
		cam1.set(CV_CAP_PROP_FRAME_HEIGHT, heigthCam);
	} else {
		string fileName = "/home/robocup/data/20150606_HTOR3_fixed.avi";
		printf( "cam1 uses file: %s\n", fileName.c_str());
		cam1.open(fileName.c_str());
	}

	ifstream cam2File("/dev/video2");
	if( cam2File.good() ) {
		printf( "cam2 uses usb camera\n");
		cam2.open(2);
		cam2.set(CV_CAP_PROP_FRAME_WIDTH, widthCam);
		cam2.set(CV_CAP_PROP_FRAME_HEIGHT, heigthCam);
	} else {
		string fileName = "/home/robocup/data/20150402_falcon_4_1.avi";
		printf( "cam2 uses file: %s\n", fileName.c_str());
		cam2.open(fileName.c_str());
	}

	ifstream cam3File("/dev/video3");
	if( cam3File.good() ) {
		printf( "cam3 uses usb camera\n");
		cam3.open(3);
		cam3.set(CV_CAP_PROP_FRAME_WIDTH, widthCam);
		cam3.set(CV_CAP_PROP_FRAME_HEIGHT, heigthCam);
	} else {
		string fileName = "/home/robocup/data/20150625_falcon_3_0.avi";
		printf( "cam3 uses file: %s\n", fileName.c_str());
		cam3.open(fileName.c_str());
	}

	ifstream cam4File("/dev/video4");
	if( cam4File.good() ) {
		printf( "cam4 uses usb camera\n");
		cam4.open(4);
		cam4.set(CV_CAP_PROP_FRAME_WIDTH, widthCam);
		cam4.set(CV_CAP_PROP_FRAME_HEIGHT, heigthCam);
	} else {
		string fileName = "/home/robocup/data/20150402_falcon_1_1.avi";
		printf( "cam4 uses file: %s\n", fileName.c_str());
		cam4.open(fileName.c_str());
	}

	ifstream cam5File("/dev/video5");
	if( cam5File.good() ) {
		printf( "cam5 uses usb camera\n");
		cam5.open(5);
		cam5.set(CV_CAP_PROP_FRAME_WIDTH, widthCam);
		cam5.set(CV_CAP_PROP_FRAME_HEIGHT, heigthCam);
	} else {
		string fileName = "/home/robocup/data/bld51_april21.avi";
		printf( "cam5 uses file: %s\n", fileName.c_str());
		cam5.open(fileName.c_str());
	}

	ifstream cam6File("/dev/video6");
	if( cam6File.good() ) {
		printf( "cam6 uses usb camera\n");
		cam6.open(6);
		cam6.set(CV_CAP_PROP_FRAME_WIDTH, widthCam);
		cam6.set(CV_CAP_PROP_FRAME_HEIGHT, heigthCam);
	} else {
		string fileName = "/home/robocup/data/20150212_falcon_3_1.avi";
		printf( "cam6 uses file: %s\n", fileName.c_str());
		cam6.open(fileName.c_str());
	}

	ifstream cam7File("/dev/video7");
	if( cam7File.good() ) {
		printf( "cam7 uses usb camera\n");
		cam7.open(7);
		cam7.set(CV_CAP_PROP_FRAME_WIDTH, widthCam);
		cam7.set(CV_CAP_PROP_FRAME_HEIGHT, heigthCam);
	} else {
		string fileName = "/home/robocup/data/20150409_falcon_5_0.avi";
		printf( "cam7 uses file: %s\n", fileName.c_str());
		cam7.open(fileName.c_str());
	}

	printf("cam 0: width %d height %d\n", (int) cam0.get(CV_CAP_PROP_FRAME_WIDTH), (int) cam0.get(CV_CAP_PROP_FRAME_HEIGHT));
	// cout << "fourcc:" << cam0.get(CV_CAP_PROP_FOURCC) << endl; // does not work
	printf("cam 1: width %d height %d\n", (int) cam1.get(CV_CAP_PROP_FRAME_WIDTH), (int) cam1.get(CV_CAP_PROP_FRAME_HEIGHT));
	printf("cam 2: width %d height %d\n", (int) cam2.get(CV_CAP_PROP_FRAME_WIDTH), (int) cam2.get(CV_CAP_PROP_FRAME_HEIGHT));
	printf("cam 3: width %d height %d\n", (int) cam3.get(CV_CAP_PROP_FRAME_WIDTH), (int) cam3.get(CV_CAP_PROP_FRAME_HEIGHT));
	printf("cam 4: width %d height %d\n", (int) cam4.get(CV_CAP_PROP_FRAME_WIDTH), (int) cam4.get(CV_CAP_PROP_FRAME_HEIGHT));
	printf("cam 5: width %d height %d\n", (int) cam5.get(CV_CAP_PROP_FRAME_WIDTH), (int) cam5.get(CV_CAP_PROP_FRAME_HEIGHT));
	printf("cam 6: width %d height %d\n", (int) cam6.get(CV_CAP_PROP_FRAME_WIDTH), (int) cam6.get(CV_CAP_PROP_FRAME_HEIGHT));
	printf("cam 7: width %d height %d\n", (int) cam7.get(CV_CAP_PROP_FRAME_WIDTH), (int) cam7.get(CV_CAP_PROP_FRAME_HEIGHT));

	pthread_t thread0, thread1, thread2, thread3, thread4, thread5, thread6, thread7;
	int  iret0, iret1, iret2, iret3, iret4, iret5, iret6, iret7;
	handleT *ip0, *ip1, *ip2, *ip3, *ip4, *ip5, *ip6, *ip7;
	ip0 = (handleT *) malloc( sizeof( handleT) );
	ip1 = (handleT *) malloc( sizeof( handleT) );
	ip2 = (handleT *) malloc( sizeof( handleT) );
	ip3 = (handleT *) malloc( sizeof( handleT) );
	ip4 = (handleT *) malloc( sizeof( handleT) );
	ip5 = (handleT *) malloc( sizeof( handleT) );
	ip6 = (handleT *) malloc( sizeof( handleT) );
	ip7 = (handleT *) malloc( sizeof( handleT) );
	ip0->id = 0;
	ip1->id = 1;
	ip2->id = 2;
	ip3->id = 3;
	ip4->id = 4;
	ip5->id = 5;
	ip6->id = 6;
	ip7->id = 7;

	while (1) {
		iret0 = pthread_create( &thread0, NULL, processCamera, (void*) ip0);
		if( iret0 ) { fprintf(stderr,"Error - pthread_create() return code: %d\n",iret0); exit(EXIT_FAILURE); }

		iret1 = pthread_create( &thread1, NULL, processCamera, (void*) ip1);
		if( iret1 ) { fprintf(stderr,"Error - pthread_create() return code: %d\n",iret1); exit(EXIT_FAILURE); }

		iret2 = pthread_create( &thread2, NULL, processCamera, (void*) ip2);
		if( iret2 ) { fprintf(stderr,"Error - pthread_create() return code: %d\n",iret2); exit(EXIT_FAILURE); }

		iret3 = pthread_create( &thread3, NULL, processCamera, (void*) ip3);
		if( iret3 ) { fprintf(stderr,"Error - pthread_create() return code: %d\n",iret3); exit(EXIT_FAILURE); }

		iret4 = pthread_create( &thread4, NULL, processCamera, (void*) ip4);
		if( iret4 ) { fprintf(stderr,"Error - pthread_create() return code: %d\n",iret4); exit(EXIT_FAILURE); }

		iret5 = pthread_create( &thread5, NULL, processCamera, (void*) ip5);
		if( iret5 ) { fprintf(stderr,"Error - pthread_create() return code: %d\n",iret5); exit(EXIT_FAILURE); }

		iret6 = pthread_create( &thread6, NULL, processCamera, (void*) ip6);
		if( iret6 ) { fprintf(stderr,"Error - pthread_create() return code: %d\n",iret6); exit(EXIT_FAILURE); }

		iret7 = pthread_create( &thread7, NULL, processCamera, (void*) ip7);
		if( iret7 ) { fprintf(stderr,"Error - pthread_create() return code: %d\n",iret7); exit(EXIT_FAILURE); }

		pthread_join( thread0, NULL);
		pthread_join( thread1, NULL);
		pthread_join( thread2, NULL);
		pthread_join( thread3, NULL);
		pthread_join( thread4, NULL);
		pthread_join( thread5, NULL);
		pthread_join( thread6, NULL);
		pthread_join( thread7, NULL);

		imshow("press q or esc to quit", combined);

		key = (char) waitKey(1); //delay N millis, usually long enough to display and capture input
		switch (key) {
			case 'q':
			case 'Q':
			case 27: //escape key
				return 0;
			default:
				break;
		}

		if( ! cam0File.good() ) {
			if( cam0.get(CV_CAP_PROP_POS_FRAMES) >= ( cam0.get(CV_CAP_PROP_FRAME_COUNT) - 1 )) {
				cam0.set(CV_CAP_PROP_POS_FRAMES, 0);
				printf("restart cam 0\n");
			}
		}
		if( ! cam1File.good() ) {
			if( cam1.get(CV_CAP_PROP_POS_FRAMES) >= ( cam1.get(CV_CAP_PROP_FRAME_COUNT) - 1 )) {
				cam1.set(CV_CAP_PROP_POS_FRAMES, 0);
				printf("restart cam 1\n");
			}
		}
		if( ! cam2File.good() ) {
			if( cam2.get(CV_CAP_PROP_POS_FRAMES) >= ( cam2.get(CV_CAP_PROP_FRAME_COUNT) - 1 )) {
				cam2.set(CV_CAP_PROP_POS_FRAMES, 0);
				printf("restart cam 2\n");
			}
		}
		if( ! cam3File.good() ) {
			if( cam3.get(CV_CAP_PROP_POS_FRAMES) >= ( cam3.get(CV_CAP_PROP_FRAME_COUNT) - 1 )) {
				cam3.set(CV_CAP_PROP_POS_FRAMES, 0);
				printf("restart cam 3\n");
			}
		}
		if( ! cam4File.good() ) {
			if( cam4.get(CV_CAP_PROP_POS_FRAMES) >= ( cam4.get(CV_CAP_PROP_FRAME_COUNT) - 1 )) {
				cam4.set(CV_CAP_PROP_POS_FRAMES, 0);
				printf("restart cam 4\n");
			}
		}
		if( ! cam5File.good() ) {
			if( cam5.get(CV_CAP_PROP_POS_FRAMES) >= ( cam5.get(CV_CAP_PROP_FRAME_COUNT) - 1 )) {
				cam5.set(CV_CAP_PROP_POS_FRAMES, 0);
				printf("restart cam 5\n");
			}
		}
		if( ! cam6File.good() ) {
			if( cam6.get(CV_CAP_PROP_POS_FRAMES) >= ( cam6.get(CV_CAP_PROP_FRAME_COUNT) - 1 )) {
				cam6.set(CV_CAP_PROP_POS_FRAMES, 0);
				printf("restart cam 6\n");
			}
		}
		if( ! cam7File.good() ) {
			if( cam7.get(CV_CAP_PROP_POS_FRAMES) >= ( cam7.get(CV_CAP_PROP_FRAME_COUNT) - 1 )) {
				cam7.set(CV_CAP_PROP_POS_FRAMES, 0);
				printf("restart cam 7\n");
			}
		}

		fpsCounter++;
		double currentTime = (double)getTickCount();
		if( ( currentTime - previousTime)/tickFrequency > 1.0 ) {
		    printf("fps %.3f\n", fpsCounter/1.0);
		    fpsCounter = 0;
		    previousTime += 1.0 * tickFrequency;
		}
	} // while

	cam0.release();
	cam1.release();
	cam2.release();
	cam3.release();
	cam4.release();
	cam5.release();
	cam6.release();
	cam7.release();
	return 0;
}
