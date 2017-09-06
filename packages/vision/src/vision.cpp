 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2014-2016 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#include "vision.hpp"

#include <cerrno>
#include <fcntl.h>    /* For O_RDWR */
#include <pwd.h>
#include <unistd.h>

using namespace std;
using namespace cv;

visionLibrary::visionLibrary( int robotIdArg, bool guiEnabled, int skipFrame ) {
	this->guiEnabled = guiEnabled;

	frameCurrent = 0;
	useCamera = false;
	previousSendTime = 0.0;

	robotId = 0;
	if( robotIdArg != 0 ) {
		robotId = robotIdArg;
	} else {
		// if robotId not provided as command line argument, use the TURTLE5K_ROBOTNUMBER environment variable
	    if( getenv("TURTLE5K_ROBOTNUMBER") != NULL ) {
			robotId = atoi(getenv("TURTLE5K_ROBOTNUMBER"));
			printf("INFO      : using environment variable TURTLE5K_ROBOTNUMBER to set robot id to %d\n", robotId);
	    }
	}

    if( robotId < 0 || robotId > 7 ) {
		printf("ERROR     : robotId %d is out of range 1 to 6\n", robotId);
		exit( EXIT_FAILURE );
    }

	conf = new configurator(robotId);

	string videoSource;
	if( conf->getVideoDevice() >= 0 ) {
		capture.open(conf->getVideoDevice());
		if( ! capture.isOpened() ) {
			printf("ERROR   : %s /dev/video%d\n", strerror(errno), conf->getVideoDevice() );
			exit( EXIT_FAILURE );
		}
		capture.set(CV_CAP_PROP_FRAME_WIDTH, 1280);//TODO configuration
		capture.set(CV_CAP_PROP_FRAME_HEIGHT, 720);//TODO configuration
		useCamera = true;
	} else {
		string videoFile = "empty";
		// The optical system of the robots differs. The robotID is used to select the test video recorded with that optical system
		// The configurator also uses the robotID to correct correction factors
		if( robotId == 1 ) {
			// videoFile = "/home/robocup/data/20150402_falcon_1_1.avi";
			// videoFile = "/home/robocup/data/20150625_falcon_1_0_chessboard.avi";
			// videoFile = "/home/robocup/data/bld66_jun24_falcon1_2.avi"; // bright sun spots
			// videoFile = "/home/robocup/data/20160508_goal_lightest_focus_95_falcon1_0.avi";
			// videoFile = "/home/robocup/data/20170427_robot1_coimbra_focus_95.avi";
			videoFile = "/home/robocup/data/20170430_robot1_coimbra_focus_95_cambada_purple_cyan_carpe_cyan.avi";
		} else if( robotId == 2 ) {
			// videoFile = "/home/robocup/data/20150625_falcon_2_0_chessboard.avi";
			// videoFile = "/home/robocup/data/20150625_falcon_2.0.avi";
			// videoFile = "/home/robocup/data/20160507_falcon_2.0_focus_85_goal_test.avi";
			videoFile = "/home/robocup/data/20160609_robot2_with_ball_in_ball_handler.avi";
		} else if( robotId == 3 ) {
			// videoFile = "/home/robocup/data/20150212_falcon_3_1.avi";
			// videoFile = "/home/robocup/data/20150625_falcon_3_0_chessboard.avi";
			videoFile = "/home/robocup/data/20150625_falcon_3_0.avi";
		} else if( robotId == 4 ) {
			// videoFile = "/home/robocup/data/20150625_falcon_4_0_chessboard.avi";
			// videoFile = "/home/robocup/data/20160505_ballecho_falcon4_1.avi";
			// videoFile = "/home/robocup/data/20160505_ball_nearby_falcon4_2.avi";
			// videoFile = "/home/robocup/data/20160609_robot4_ball_in_ball_handler.avi";
			videoFile = "/home/robocup/data/20160908_robot4_locht_focus_85.avi";
		} else if( robotId == 5 ) {
			// videoFile = "/home/robocup/data/20150625_falcon_5_0_chessboard.avi";
			// videoFile = "/home/robocup/data/20150409_falcon_5_0.avi"; // techunited
			// videoFile = "/home/robocup/data/20150408_falcon_5_0.avi"; // chessboard
			// videoFile = "/home/robocup/data/20150606_HTOR3_fixed.avi"; // lots of sun spots
			// videoFile = "/home/robocup/data/20160402_evolution_falcon_5.2.avi"; // lots of sun spots
			// videoFile = "/home/robocup/data/20160609_robot5_with_ball_in_handler.avi";
			videoFile = "/home/robocup/data/20161006_robot5_locht_focus100.avi";
		} else if( robotId == 6 ) {
			videoFile = "/home/robocup/data/20150408_falcon_6_0.avi"; // chessboard
		} else if( robotId == 7 ) {
			videoFile = "/home/robocup/data/20150614_pannekoek_center1.avi"; // center of field
		}

		printf("INFO      : No Camera Found, use video file %s\n", videoFile.c_str());

		// Check if the select movie exist on the file system
		std::ifstream myMovieFile(videoFile.c_str());
		if( ! myMovieFile.good() ) {
			printf("ERROR     : video file %s could not be opened!\n", videoFile.c_str());
			printf("ERROR     : Download content from: https://drive.google.com/folderview?id=0Byfy7jb9WRXfRjh6YkJwVHRxM2M#list\n");
			exit( EXIT_FAILURE );
		}
		capture.open(videoFile);
		if( skipFrame >= 0 ) {
			capture.set(CV_CAP_PROP_POS_FRAMES, skipFrame );
		}
	}

	prep = new preprocessor(conf);
	ballDet = new ballDetection(conf, prep, ballType);
	cyanDet = new ballDetection(conf, prep, cyanType);
	magentaDet = new ballDetection(conf, prep, magentaType);
	ballPos = new ballPossession(conf, prep); // determine ball in ball handlers
	linePoint = new linePointDetection(conf, prep);
	rFloor = new robotFloor(conf);
	detPos = new determinePosition(conf, linePoint, prep, rFloor);
	loc = new localization(conf, detPos, linePoint, prep);
	obstDet = new obstacleDetection(conf, prep);
	view = new viewer(ballDet, ballPos, conf, cyanDet, detPos, linePoint, loc, magentaDet, obstDet, prep, rFloor);
	diag = new cFrameDiagnostics(conf, rFloor, view);
	multSend = new multicastSend(ballDet, ballPos, conf, cyanDet, detPos, linePoint, loc, magentaDet, obstDet, prep, rFloor);

	conf->init(rFloor->getWidth(), rFloor->getHeight(), guiEnabled);

}

void visionLibrary::attach(observer *obs)
{
    detPos->attach(obs);
    ballDet->attach(obs);
    obstDet->attach(obs);
    ballPos->attach(obs);
}

bool visionLibrary::update( ) {
	conf->update( );
	if( ! useCamera ) { frameCurrent = capture.get(CV_CAP_PROP_POS_FRAMES); } // get current file pointer, if camera then ignore

	capture >> frameRaw; // get a frame from the camera or file, re-read every cycle
	prep->update(frameRaw);

	// the ball detection, ball possession and obstacle detection need to wait until the preprocessing has been finished
	ballDetThread = thread(&ballDetection::update, ballDet, true);
	if( ! loc->getBusy() ) { // only start new localization thread when previous thread was finished
		if( locThread.joinable() ) { // this should be almost always the case
			locThread.join(); // be sure to join the previous tread before we start a new one
			double uptime = prep->getUptime();
			double multicastSendPeriod = 1.0 / conf->getMulticast().frequency;
			if( uptime > ( previousSendTime + multicastSendPeriod) ) {
				multSend->stats(); // used to invalidate other packets, send as first packet
				multSend->goodEnoughLoc(); // goodEnough is used by the others, except stats(), send as second packet
				multSend->floorLinePoints();
				multSend->locList();
				multSend->ballList(TYPE_BALLDETECTION);
				multSend->ballList(TYPE_CYANDETECTION);
				multSend->ballList(TYPE_MAGENTADETECTION);
				multSend->obstaclelList();
				// increase previousSendTime for the next send
				while( uptime > ( previousSendTime + multicastSendPeriod) ) {
					previousSendTime+=multicastSendPeriod;
				}
			}
		}
		loc->setBusy(); // semaphore
		locThread = thread(&localization::update, loc);
	}
	if( ! view->getBusy() ) { // only start new viewer thread when previous thread has finished
		if( viewerThread.joinable() ) { // this should be almost always the case
			viewerThread.join(); // be sure to join the previous tread before we start a new one
		}
		view->setBusy(); // semaphore
		viewerThread = thread(&viewer::update, view);
	}
	if( ! diag->getBusy() ) { // only start new diag thread when previous thread has finished
		if( diagThread.joinable() ) { // this should be almost always the case
			diagThread.join(); // be sure to join the previous tread before we start a new one
		}
		diag->setBusy(); // semaphore
		diagThread = thread(&cFrameDiagnostics::update, diag);
	}
	if( ! cyanDet->getBusy() ) { // only start new cyan detection thread when previous thread has finished
		if( cyanDetThread.joinable() ) { // this should be almost always the case
			cyanDetThread.join(); // be sure to join the previous tread before we start a new one
		}
		cyanDet->setBusy(); // semaphore
		cyanDetThread = thread(&ballDetection::update, cyanDet, true);
	}
	if( ! magentaDet->getBusy() ) { // only start new magenta detection thread when previous thread has finished
		if( magentaDetThread.joinable() ) { // this should be almost always the case
			magentaDetThread.join(); // be sure to join the previous tread before we start a new one
		}
		magentaDet->setBusy(); // semaphore
		magentaDetThread = thread(&ballDetection::update, magentaDet, true);
	}
	if( ! ballPos->getBusy() ) { // only start new ball possession detection thread when previous thread has finished
		if( ballPosThread.joinable() ) { // this should be almost always the case
			ballPosThread.join(); // be sure to join the previous tread before we start a new one
		}
		ballPos->setBusy(); // semaphore
		ballPosThread = thread(&ballPossession::update, ballPos);
	}
	if( ! obstDet->getBusy() ) { // only start new obstacle detection thread when previous thread has finished
		if( obstDetThread.joinable() ) { // this should be almost always the case
			obstDetThread.join(); // be sure to join the previous tread before we start a new one
		}
		obstDet->setBusy(); // semaphore
		obstDetThread = thread(&obstacleDetection::update, obstDet, robotId);
	}

	ballDetThread.join();

	rFloor->update(); // used when swap from normal mode to calibration mode

	int key = view->getKey( );
	if( key == 27 || key == 'q' ) // escape or q
	{
	    return false;
	}
	if( ! useCamera ) {
		// following keys are only available when video is read from file instead of camera (which is when frameCurrent >= 0)
		if( key == 'r' ) { capture.set(CV_CAP_PROP_POS_FRAMES, 0 ); } // r
		if( key == 's'|| key == 65361 || key == '[' ) { // left arrow
			frameCurrent = frameCurrent - 10;
			if( frameCurrent < 0 ) { frameCurrent = 0; }
			capture.set(CV_CAP_PROP_POS_FRAMES, frameCurrent );
		}
		if( key == 'f' || key == 65362 ) { // up arrow
			frameCurrent = frameCurrent + 60;
			if( frameCurrent >= (capture.get(CV_CAP_PROP_FRAME_COUNT) - 1) ) { frameCurrent = 0; }
			capture.set(CV_CAP_PROP_POS_FRAMES, frameCurrent );
		}
		if( key == 'd' || key == 65363 || key == ']' ) { // right arrow
			frameCurrent = frameCurrent + 10;
			if( frameCurrent >= (capture.get(CV_CAP_PROP_FRAME_COUNT) - 1) ) { frameCurrent = 0; }
			capture.set(CV_CAP_PROP_POS_FRAMES, frameCurrent );
		}
		if( key == 'a' || key == 65364 ) { // down arrow
			frameCurrent = frameCurrent - 60;
			capture.set(CV_CAP_PROP_POS_FRAMES, frameCurrent );
			if( frameCurrent < 0 ) { frameCurrent = 0; }
		}
		if( key == 'x' ) {
			frameCurrent = frameCurrent + 1;
			if( frameCurrent >= (capture.get(CV_CAP_PROP_FRAME_COUNT) - 1) ) { frameCurrent = 0; }
			capture.set(CV_CAP_PROP_POS_FRAMES, frameCurrent );
		}
		if( key == 'z' ) {
			frameCurrent = frameCurrent - 1;
			capture.set(CV_CAP_PROP_POS_FRAMES, frameCurrent );
			if( frameCurrent < 0 ) { frameCurrent = 0; }
		}

		if( view->getPause() ) { capture.set(CV_CAP_PROP_POS_FRAMES, frameCurrent ); }

		if( frameCurrent >= ( capture.get(CV_CAP_PROP_FRAME_COUNT) - 1 )) {
			frameCurrent = 0;
			capture.set(CV_CAP_PROP_POS_FRAMES, 0);
		}
	}

	return true;
}
