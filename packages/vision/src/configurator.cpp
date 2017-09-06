 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2014-2017 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#include "configurator.hpp"

#include <unistd.h>
#include <pwd.h>

using namespace cv;
using namespace std;

configurator::configurator(int robot)
{
	zLin = 0;
	erodeSlider = 0;
	guiEnabled = false;
	tmpRos = 0.0;
	dilateSlider = 0;

	struct passwd *pw = getpwuid(getuid());
	std::string configFile("");
	configFile.append(pw->pw_dir);
	configFile.append("/falcons/code/packages/vision/vision.yaml");
	FileStorage fs(configFile, FileStorage::READ);

	FileNode global = fs["global"];
	remote = false; // default not used for remote viewer
	centerPixel = (int)global["centerPixel"]; // this is the exact center, so range is from pixel 0 to pixel 680 = 681 pixels !
	viewPixels = 2*centerPixel + 1;
	centerPoint = Point(centerPixel, centerPixel);

	multicast.frequency = (int)(global["multicastFrequency"]);
	multicast.ip = (string)(global["multicastIp"]);
	multicast.port = (int)(global["multicastPort"]);

	goodEnough.angleToPixelRatio = (double)(global["goodEnoughAngleToPixelRatio"]);
	goodEnough.xDeltaKeepThreshold = (double)(global["goodEnoughXDeltaKeepThreshold"]);
	goodEnough.yDeltaKeepThreshold = (double)(global["goodEnoughYDeltaKeepThreshold"]);
	goodEnough.rzDeltaKeepThreshold = (int)(global["goodEnoughRzDeltaKeepThreshold"]);
	goodEnough.keepWatchdog = (int)(global["goodEnoughKeepWatchdog"]);
	goodEnough.newMinAge = (int)(global["goodEnoughNewMinAge"]);
	goodEnough.newThreshold = (double)(global["goodEnoughNewThreshold"]);
	scoreThresHold = (double)(global["scoreThresHold"]);
	camera.latencyOffset = (int)(global["cameraTimingOffset"]);

	manual.x = 0; // set in configurator::init with values from robot floor class
	manual.y = 0; // set in configurator::init with values from robot floor class
	manual.rz = 0; // set in configurator::init

	this->manualMode = false;
	this->calibrateMode = false;
	solver.xStep = 150;
	solver.yStep = 150;
	solver.rzStep = 30; // when using compass, otherwise 100
	solver.maxCount = 400;
	solver.score = 20;
	blurPixels = 40;

	this->robot = robot;

	FileNode specific;
	if( robot == 1 ) { specific = fs["robot1"];
   	} else if( robot == 2 ) { specific = fs["robot2"];
   	} else if( robot == 3 ) { specific = fs["robot3"];
   	} else if( robot == 4 ) { specific = fs["robot4"];
	} else if( robot == 5 ) { specific = fs["robot5"];
	} else if( robot == 6 ) { specific = fs["robot6"];
	} else if( robot == 7 ) { specific = fs["robot7"];
	} else if( robot == 8 ) { specific = fs["robot8"];
	} else {
		std::cerr << "vision: Robot " << robot << " has no configuration" << std::endl;
		exit(1);
	}
	xOpticalBias = (int)(specific["xOpticalBias"]); // correct xCenter is 60, range is 0 to 120 = 121 pixels
	yOpticalBias = (int)(specific["yOpticalBias"]); // correct yCenter is 20, range is 0 to 39 = 40 pixels
	possession.yOffset = (int)(specific["yOffsetBallPossession"]);

	exportOffsetInt.x = (int)(specific["xExportOffset"]);
	exportOffsetInt.y = (int)(specific["yExportOffset"]);
	exportOffsetInt.rz = (int)(specific["rzExportOffset"]);
	exportOffsetInt.rzBall = (int)(specific["rzBallExportOffset"]); // use this ball rz correction also for cyan and magenta
	exportOffsetInt.rzObstacle = (int)(specific["rzObstacleExportOffset"]);

	// APOX todo: add keeper frame sizes over here instead of radius minimum
	cameraRadiusObstacleMin = (int)(specific["cameraRadiusObstacleMin"]); // exclude center area for obstacles, but not for ball to be able to see the ball "in" shooter
	cameraRadiusObstacleMax = (int)(specific["cameraRadiusObstacleMax"]); // exclude distorted area at end of parabolic mirror for obstacles (rapid increase meter/pixel ratio)
	cameraRadiusBallMax = (int)(specific["cameraRadiusBallMax"]); // exclude distorted area at end of parabolic mirror for balls
	cameraRadiusLineMax = (int)(specific["cameraRadiusLineMax"]); // exclude distorted area at end of parabolic mirror for lines
	// deWarp values from cameraCalibrate.py and chessBoard video
	// the deWarp formula : meters = a * tan ( b*pixel ) + c * pixel + d * pixel^2
	// deWarp values from cameraCalibrate.py and chessBoard video
	deWarp.pixelPow1 = (double)(specific["deWarp.pixelPow1"]);
	deWarp.pixelPow2 = (double)(specific["deWarp.pixelPow2"]);
	deWarp.outSideTan = (double)(specific["deWarp.outSideTan"]);
	deWarp.inSideTan = (double)(specific["deWarp.inSideTan"]);

	// project additional goal posts on the field
	goalPostIn = (int)(specific["goalPostIn"]);
	goalPostOut = (int)(specific["goalPostOut"]);

	// these colors could also be different for each robot, but probably not
	// create color window for white lines
	FileNode line = fs["line"];
	lineSt.hue.center = (int)(line["hue.center"]); // hue range from 0 to 179
	lineSt.hue.delta = (int)(line["hue.delta"]); // white has all "colors", so full range
	lineSt.hue.min = 0; // calculated from lineSt.hue.center and lineSt.hue.delta
	lineSt.hue.max = 0; // calculated from lineSt.hue.center and lineSt.hue.delta
	lineSt.sat.min = (int)(line["sat.min"]);
	lineSt.sat.max = (int)(line["sat.max"]); // white has low saturation
	lineSt.val.min = (int)(line["val.min"]); // highly depending on illumination
	lineSt.calVal.min = (int)(line["calVal.min"]); // calibration with black and white chessboard floor requires different min value
	lineSt.val.max = (int)(line["val.max"]); // not upto 255 because of light source reflections
	lineSt.linePointsMinimal = (int)(line["linePointsMinimal"]); // amount of points to be classified as line
	lineSt.linePointsNumber = (int)(line["linePointsNumber"]); // maximal amount of line points used for search
	lineSt.lineRejectWidth = (int)(line["lineRejectWidth"]); // wider lines will be rejected
	lineSt.lineHoleWidth = (int)(line["lineHoleWidth"]); // combine parts of lines e.g. because of dirty spots
	lineSt.erodeSlider = (int)(line["erodeSlider"]); // used find sun spot (large wide area on field because of bright sunlight)
	lineSt.dilateSlider = (int)(line["dilateSlider"]); // used find sun spot
	lineSt.size = 0; // not used
	lineSt.sizeMax = 0; // not used
	lineSt.sunSpotWidth = (int)(line["sunSpotWidth"]); // minimal width of sun spot
	lineSt.sunSpotHeight = (int)(line["sunSpotHeigth"]); // minimal height of sun spot
	lineSt.sunSpotRatio = (int)(line["sunSpotRatio"]); // ( ratio * real sun width * real sun height ) / 100 = minimal real sun spot pixels, so 100 is completely filled
	if( robot == 1 ) {
		lineSt.goalRearMask = (int)(line["goalRearMask"]); // amount of pixels blocked by robot1 to prevent issues with the goal net
	} else {
		lineSt.goalRearMask = 0; // not used by other robots
	}
	lineSt.goalRearDistance = (int)(line["goalRearDistance"]); // maximal pixel distance from robot to goal line where mask is allowed
	lineSt.goalRearAngle = (int)(line["goalRearAngle"]); // maximal pixel distance from robot to goal line where mask is allowed
	lineSt.goalRearWatchdog = (int)(line["goalRearWatchdog"]); // watchdog (in frames) removing mask when no new good enough position has been provided

	// create color window for floor, this is used as input for the dynamic color / brightness calibration
	FileNode floor = fs["floor"];
	floorSt.hue.center = (int)(floor["hue.center"]); // hue range from 0 to 179
	floorSt.hue.delta = (int)(floor["hue.delta"]);
	floorSt.hue.min = 0; // calculated from floorSt.hue.center and floorSt.hue.delta
	floorSt.hue.max = 0; // calculated from floorSt.hue.center and floorSt.hue.delta
	floorSt.sat.min = (int)(floor["sat.min"]);
	floorSt.sat.max = (int)(floor["sat.max"]);
	floorSt.val.min = (int)(floor["val.min"]);
	floorSt.val.max = (int)(floor["val.max"]); // not upto 255 because of light source reflections
	floorSt.calVal.min = 0; // not used
	floorSt.linePointsMinimal = 0; // not used
	floorSt.linePointsNumber = 0; // not used
	floorSt.lineRejectWidth = 0; // not used
	floorSt.lineHoleWidth = 0; // not used
	floorSt.erodeSlider = 0; // not used
	floorSt.dilateSlider = 0; // not used
	floorSt.size = 0; // not used
	floorSt.sizeMax = 0; // not used
	floorSt.sunSpotWidth = 0; // not used
	floorSt.sunSpotHeight = 0; // not used
	floorSt.sunSpotRatio = 0; // not used

	// create color window for yellow ball
	FileNode ballConf = fs["ball"];
	ball.hue.center = (int)(ballConf["hue.center"]); // hue range from 0 to 179
	ball.hue.delta = (int)(ballConf["hue.delta"]);
	ball.hue.min = 0; // calculated from ball.hue.center and ball.hue.delta
	ball.hue.max = 0; // calculated from ball.hue.center and ball.hue.delta
	ball.sat.min = (int)(ballConf["sat.min"]); // ball at edge of round viewer loses color saturation
	ball.sat.max = (int)(ballConf["sat.max"]);
	ball.val.min = (int)(ballConf["val.min"]);
	ball.val.max = (int)(ballConf["val.max"]);
	ball.calVal.min = 0; // not used
	ball.linePointsMinimal = 0; // not used
	ball.linePointsNumber = 0; // not used
	ball.lineRejectWidth = 0; // not used
	ball.lineHoleWidth = 0; // not used
	ball.erodeSlider = (int)(ballConf["erodeSlider"]);
	ball.dilateSlider = (int)(ballConf["dilateSlider"]);
	ball.size = (int)(ballConf["size"]); // minimal amount of ball pixels after dilate step (in round viewer)
	ball.sizeMax = (int)(ballConf["sizeMax"]); // maximal amount of ball pixels after dilate step (in round viewer), e.g. floor reflection or yellow floor outside field
	ball.sunSpotWidth = 0; // not used
	ball.sunSpotHeight = 0; // not used
	ball.sunSpotRatio = 0; // not used

	// create color window for the ball possession
	FileNode possessionConf = fs["possession"];
	// the possession.yOffset is set at the robot specific section
	possession.hue.center = (int)(possessionConf["hue.center"]); // hue range from 0 to 179
	possession.hue.delta = (int)(possessionConf["hue.delta"]);
	possession.hue.min = possession.hue.center - (possession.hue.delta-1)/2; // -1 for slider granularity
	possession.hue.max = possession.hue.center + possession.hue.delta/2;
	possession.sat.min = (int)(possessionConf["sat.min"]); // ball in handler loses color saturation because poor light
	possession.sat.max = (int)(possessionConf["sat.max"]);
	possession.val.min = (int)(possessionConf["val.min"]);
	possession.val.max = (int)(possessionConf["val.max"]);
	possession.width = (int)(possessionConf["width"]);
	possession.height = (int)(possessionConf["height"]);
	possession.threshold = (int)(possessionConf["threshold"]);

	// create color window for the black robots
	FileNode obstacleConf = fs["obstacle"];
	obstacle.hue.center =  (int)(obstacleConf["hue.center"]); // hue range from 0 to 179
	obstacle.hue.delta = (int)(obstacleConf["hue.delta"]);
	obstacle.hue.min = 0; // calculated from obstacle.hue.center and obstacle.hue.delta
	obstacle.hue.max = 0; // calculated from obstacle.hue.center and obstacle.hue.delta
	obstacle.sat.min = (int)(obstacleConf["sat.min"]);
	obstacle.sat.max = (int)(obstacleConf["sat.max"]);
	obstacle.val.min = (int)(obstacleConf["val.min"]);
	obstacle.val.max = (int)(obstacleConf["val.max"]);
	obstacle.calVal.min = 0; // not used
	obstacle.linePointsMinimal = 0; // not used
	obstacle.linePointsNumber = 0; // not used
	obstacle.lineRejectWidth = 0; // not used
	obstacle.lineHoleWidth = 0; // not used
	obstacle.erodeSlider = (int)(obstacleConf["erodeSlider"]);
	obstacle.dilateSlider = (int)(obstacleConf["dilateSlider"]);
	obstacle.size = (int)(obstacleConf["size"]); // amount object dilate pixel in round viewer
	obstacle.sizeMax = 0; // not used
	obstacle.sunSpotWidth = 0; // not used
	obstacle.sunSpotHeight = 0; // not used
	obstacle.sunSpotRatio = 0; // not used

	// create color window for the cyan label on robot
	FileNode cyanConf = fs["cyan"];
	cyan.hue.center = (int)(cyanConf["hue.center"]); // hue range from 0 to 179
	cyan.hue.delta = (int)(cyanConf["hue.delta"]);
	cyan.hue.min = 0; // calculated from cyan.hue.center and cyan.hue.delta
	cyan.hue.max = 0; // calculated from cyan.hue.center and cyan.hue.delta
	cyan.sat.min = (int)(cyanConf["sat.min"]); // cyan at edge of round viewer loses color saturation
	cyan.sat.max = (int)(cyanConf["sat.max"]);
	cyan.val.min = (int)(cyanConf["val.min"]);
	cyan.val.max = (int)(cyanConf["val.max"]);
	cyan.calVal.min = 0; // not used
	cyan.linePointsMinimal = 0; // not used
	cyan.linePointsNumber = 0; // not used
	cyan.lineRejectWidth = 0; // not used
	cyan.lineHoleWidth = 0; // not used
	cyan.erodeSlider = (int)(cyanConf["erodeSlider"]);
	cyan.dilateSlider = (int)(cyanConf["dilateSlider"]);
	cyan.size = (int)(cyanConf["size"]); // minimal amount of cyan pixels after dilate step (in round viewer)
	cyan.sizeMax = (int)(cyanConf["sizeMax"]); // maximal amount of cyan pixels after dilate step (in round viewer), e.g. floor reflection or yellow floor outside field
	cyan.sunSpotWidth = 0; // not used
	cyan.sunSpotHeight = 0; // not used
	cyan.sunSpotRatio = 0; // not used

	// create color window for the magenta label on robot
	FileNode magentaConf = fs["magenta"];
	magenta.hue.center = (int)(magentaConf["hue.center"]); // hue range from 0 to 179
	magenta.hue.delta = (int)(magentaConf["hue.delta"]);
	magenta.hue.min = 0; // calculated from magenta.hue.center and magenta.hue.delta
	magenta.hue.max = 0; // calculated from magenta.hue.center and magenta.hue.delta
	magenta.sat.min = (int)(magentaConf["sat.min"]); // magenta at edge of round viewer loses color saturation
	magenta.sat.max = (int)(magentaConf["sat.max"]);
	magenta.val.min = (int)(magentaConf["val.min"]);
	magenta.val.max = (int)(magentaConf["val.max"]);
	magenta.calVal.min = 0; // not used
	magenta.linePointsMinimal = 0; // not used
	magenta.linePointsNumber = 0; // not used
	magenta.lineRejectWidth = 0; // not used
	magenta.lineHoleWidth = 0; // not used
	magenta.erodeSlider = (int)(magentaConf["erodeSlider"]);
	magenta.dilateSlider = (int)(magentaConf["dilateSlider"]);
	magenta.size = (int)(magentaConf["size"]); // minimal amount of magenta pixels after dilate step (in round viewer)
	magenta.sizeMax = (int)(magentaConf["sizeMax"]); // maximal amount of magenta pixels after dilate step (in round viewer), e.g. floor reflection or yellow floor outside field
	magenta.sunSpotWidth = 0; // not used
	magenta.sunSpotHeight = 0; // not used
	magenta.sunSpotRatio = 0; // not used

	// gather (default) settings for usb camera
	FileNode cameraConf = fs["camera"];
	camera.backlightCompensation = cameraConf["backlightCompensation"];
	camera.brightness = cameraConf["brightness"];
	camera.contrast = cameraConf["contrast"];
	camera.exposureAbsolute = cameraConf["exposureAbsolute"];
	camera.exposureAuto = cameraConf["exposureAuto"];
	camera.exposureAutoPriority = cameraConf["exposureAutoPriority"];
	camera.focusAbsolute = specific["focusAbsolute"]; // differs for each robot
	camera.focusAuto = cameraConf["focusAuto"];
	camera.gain = cameraConf["gain"];
	camera.led1Frequency = cameraConf["led1Frequency"];
	camera.led1Mode = cameraConf["led1Mode"];
	camera.panAbsolute = cameraConf["panAbsolute"];
	camera.powerLineFrequency = cameraConf["powerLineFrequency"];
	camera.saturation = cameraConf["saturation"];
	camera.sharpness = cameraConf["sharpness"];
	camera.tiltAbsolute = cameraConf["tiltAbsolute"];
	camera.whiteBalanceAuto = cameraConf["whiteBalanceAuto"];
	camera.whiteBalanceTemperature = cameraConf["whiteBalanceTemperature"];
	camera.zoomAbsolute = cameraConf["zoomAbsolute"];

	fs.release();
}

void configurator::init(int floorWidth, int floorHeight, bool guiEnabled = false ) {
	this->guiEnabled = guiEnabled;
	if( floorWidth%2 == 0 ) { cerr << "vision: configurator Error: floor width should be an odd number but is: " << floorWidth << endl;; exit(1); }
	if( floorHeight%2 == 0 ) { cerr << "vision: configurator Error: floor height should be an odd number but is: " << floorHeight << endl;; exit(1); }
	manual.x = (floorWidth-1)/2; // start in exact center
	manual.y = (floorHeight-1)/2;
	manual.rz = 90; // default shooter on y-axis

	if( guiEnabled ) {
		showTrackbars(floorWidth, floorHeight);
	}

	updateLineErode( lineSt.erodeSlider );
	updateLineDilate( lineSt.dilateSlider );
	updateBallErode( ball.erodeSlider );
	updateBallDilate( ball.dilateSlider );
	updateObstacleErode( obstacle.erodeSlider );
	updateObstacleDilate( obstacle.dilateSlider );
	updateCyanErode( cyan.erodeSlider );
	updateCyanDilate( cyan.dilateSlider );
	updateMagentaErode( magenta.erodeSlider );
	updateMagentaDilate( magenta.dilateSlider );
	// create line, ball and obstacle color window for initial values
	drawLineBGR();
	drawFloorBGR();
	drawBallBGR();
	drawObstacleBGR();
	drawCyanBGR();
	drawMagentaBGR();
}


void configurator::showTrackbars(int floorWidth, int floorHeight) {

	// create the slide bars
	colorWindowLine = "line";
	namedWindow(colorWindowLine, CV_WINDOW_NORMAL);
	createTrackbar( "l Hue Cent", colorWindowLine, &lineSt.hue.center, 179, &configurator::cbUpdateLineTrackbar, this ); // hue range is 0 to 179
	createTrackbar( "l Hue Delta", colorWindowLine, &lineSt.hue.delta, 180, &configurator::cbUpdateLineTrackbar, this );
	createTrackbar( "l Sat Min", colorWindowLine, &lineSt.sat.min, 255, &configurator::cbUpdateLineTrackbar, this );
	createTrackbar( "l Sat Max", colorWindowLine, &lineSt.sat.max, 255, &configurator::cbUpdateLineTrackbar, this );
	createTrackbar( "l Val Min", colorWindowLine, &lineSt.val.min, 255, &configurator::cbUpdateLineTrackbar, this );
	createTrackbar( "l CalValMin", colorWindowLine, &lineSt.calVal.min, 255, &configurator::cbUpdateLineTrackbar, this );
	createTrackbar( "l Val Max", colorWindowLine, &lineSt.val.max, 255, &configurator::cbUpdateLineTrackbar, this );
	// createTrackbar( "l Line Points", colorWindowLine, &lineSt.linePointsMinimal, 50 );
	createTrackbar( "l Line Reject", colorWindowLine, &lineSt.lineRejectWidth, 100 );
	createTrackbar( "l Line Hole", colorWindowLine, &lineSt.lineHoleWidth, 50 );
	createTrackbar( "l Erode", colorWindowLine, &lineSt.erodeSlider, 20, &configurator::cbUpdateLineErodeTrackbar, this );
	createTrackbar( "l Dilate", colorWindowLine, &lineSt.dilateSlider, 20, &configurator::cbUpdateLineDilateTrackbar, this );
	createTrackbar( "l Sun Width", colorWindowLine, &lineSt.sunSpotWidth, 500 );
	createTrackbar( "l Sun Height", colorWindowLine, &lineSt.sunSpotHeight, 400 );
	createTrackbar( "l Sun Ratio", colorWindowLine, &lineSt.sunSpotRatio, 100 );

	colorWindowFloor = "floor";
	namedWindow(colorWindowFloor, CV_WINDOW_NORMAL);
	createTrackbar( "f Hue Cent", colorWindowFloor, &floorSt.hue.center, 179, &configurator::cbUpdateFloorTrackbar, this ); // hue range is 0 to 179
	createTrackbar( "f Hue Delta", colorWindowFloor, &floorSt.hue.delta, 180, &configurator::cbUpdateFloorTrackbar, this );
	createTrackbar( "f Sat Min", colorWindowFloor, &floorSt.sat.min, 255, &configurator::cbUpdateFloorTrackbar, this );
	createTrackbar( "f Sat Max", colorWindowFloor, &floorSt.sat.max, 255, &configurator::cbUpdateFloorTrackbar, this );
	createTrackbar( "f Val Min", colorWindowFloor, &floorSt.val.min, 255, &configurator::cbUpdateFloorTrackbar, this );
	createTrackbar( "f Val Max", colorWindowFloor, &floorSt.val.max, 255, &configurator::cbUpdateFloorTrackbar, this );

	colorWindowBall = "ball";
	namedWindow(colorWindowBall, CV_WINDOW_NORMAL);
	createTrackbar( "b Hue Cent", colorWindowBall, &ball.hue.center, 179, &configurator::cbUpdateBallTrackbar, this ); // hue range is 0 to 179
	createTrackbar( "b Hue Delta", colorWindowBall, &ball.hue.delta, 180, &configurator::cbUpdateBallTrackbar, this );
	createTrackbar( "b Sat Min", colorWindowBall, &ball.sat.min, 255, &configurator::cbUpdateBallTrackbar, this );
	createTrackbar( "b Sat Max", colorWindowBall, &ball.sat.max, 255, &configurator::cbUpdateBallTrackbar, this );
	createTrackbar( "b Val Min", colorWindowBall, &ball.val.min, 255, &configurator::cbUpdateBallTrackbar, this );
	createTrackbar( "b Val Max", colorWindowBall, &ball.val.max, 255, &configurator::cbUpdateBallTrackbar, this );
	createTrackbar( "b Erode", colorWindowBall, &ball.erodeSlider, 20, &configurator::cbUpdateBallErodeTrackbar, this );
	createTrackbar( "b Dilate", colorWindowBall, &ball.dilateSlider, 20, &configurator::cbUpdateBallDilateTrackbar, this );
	createTrackbar( "b Size Min", colorWindowBall, &ball.size, 300);
	createTrackbar( "b Size Max", colorWindowBall, &ball.sizeMax, 2000);

	colorWindowPossession = "possession";
	namedWindow(colorWindowPossession, CV_WINDOW_NORMAL);
	createTrackbar( "p Hue Cent", colorWindowPossession, &possession.hue.center, 179, &configurator::cbUpdatePossessionTrackbar, this ); // hue range is 0 to 179
	createTrackbar( "p Hue Delta", colorWindowPossession, &possession.hue.delta, 180, &configurator::cbUpdatePossessionTrackbar, this );
	createTrackbar( "p Sat Min", colorWindowPossession, &possession.sat.min, 255, &configurator::cbUpdatePossessionTrackbar, this );
	createTrackbar( "p Sat Max", colorWindowPossession, &possession.sat.max, 255, &configurator::cbUpdatePossessionTrackbar, this );
	createTrackbar( "p Val Min", colorWindowPossession, &possession.val.min, 255, &configurator::cbUpdatePossessionTrackbar, this );
	createTrackbar( "p Val Max", colorWindowPossession, &possession.val.max, 255, &configurator::cbUpdatePossessionTrackbar, this );
	createTrackbar( "p y offset", colorWindowPossession, &possession.yOffset, 80 );
	createTrackbar( "p Width", colorWindowPossession, &possession.width, 120);
	createTrackbar( "p Height", colorWindowPossession, &possession.height, 100);

	colorWindowObstacle = "obstacle";
	namedWindow(colorWindowObstacle, CV_WINDOW_NORMAL);
	createTrackbar( "o Hue Cent", colorWindowObstacle, &obstacle.hue.center, 179, &configurator::cbUpdateObstacleTrackbar, this ); // hue range is 0 to 179
	createTrackbar( "o Hue Delta", colorWindowObstacle, &obstacle.hue.delta, 180, &configurator::cbUpdateObstacleTrackbar, this );
	createTrackbar( "o Sat Min", colorWindowObstacle, &obstacle.sat.min, 255, &configurator::cbUpdateObstacleTrackbar, this );
	createTrackbar( "o Sat Max", colorWindowObstacle, &obstacle.sat.max, 255, &configurator::cbUpdateObstacleTrackbar, this );
	createTrackbar( "o Val Min", colorWindowObstacle, &obstacle.val.min, 255, &configurator::cbUpdateObstacleTrackbar, this );
	createTrackbar( "o Val Max", colorWindowObstacle, &obstacle.val.max, 255, &configurator::cbUpdateObstacleTrackbar, this );
	createTrackbar( "o Erode", colorWindowObstacle, &obstacle.erodeSlider, 20, &configurator::cbUpdateObstacleErodeTrackbar, this );
	createTrackbar( "o Dilate", colorWindowObstacle, &obstacle.dilateSlider, 20, &configurator::cbUpdateObstacleDilateTrackbar, this );
	createTrackbar( "o Size", colorWindowObstacle, &obstacle.size, 4000 );

	colorWindowCyan = "cyan";
	namedWindow(colorWindowCyan, CV_WINDOW_NORMAL);
	createTrackbar( "c Hue Cent", colorWindowCyan, &cyan.hue.center, 179, &configurator::cbUpdateCyanTrackbar, this ); // hue range is 0 to 179
	createTrackbar( "c Hue Delta", colorWindowCyan, &cyan.hue.delta, 180, &configurator::cbUpdateCyanTrackbar, this );
	createTrackbar( "c Sat Min", colorWindowCyan, &cyan.sat.min, 255, &configurator::cbUpdateCyanTrackbar, this );
	createTrackbar( "c Sat Max", colorWindowCyan, &cyan.sat.max, 255, &configurator::cbUpdateCyanTrackbar, this );
	createTrackbar( "c Val Min", colorWindowCyan, &cyan.val.min, 255, &configurator::cbUpdateCyanTrackbar, this );
	createTrackbar( "c Val Max", colorWindowCyan, &cyan.val.max, 255, &configurator::cbUpdateCyanTrackbar, this );
	createTrackbar( "c Erode", colorWindowCyan, &cyan.erodeSlider, 20, &configurator::cbUpdateCyanErodeTrackbar, this );
	createTrackbar( "c Dilate", colorWindowCyan, &cyan.dilateSlider, 20, &configurator::cbUpdateCyanDilateTrackbar, this );
	createTrackbar( "c Size Min", colorWindowCyan, &cyan.size, 300);
	createTrackbar( "c Size Max", colorWindowCyan, &cyan.sizeMax, 2000);

	colorWindowMagenta = "magenta";
	namedWindow(colorWindowMagenta, CV_WINDOW_NORMAL);
	createTrackbar( "m Hue Cent", colorWindowMagenta, &magenta.hue.center, 179, &configurator::cbUpdateMagentaTrackbar, this ); // hue range is 0 to 179
	createTrackbar( "m Hue Delta", colorWindowMagenta, &magenta.hue.delta, 180, &configurator::cbUpdateMagentaTrackbar, this );
	createTrackbar( "m Sat Min", colorWindowMagenta, &magenta.sat.min, 255, &configurator::cbUpdateMagentaTrackbar, this );
	createTrackbar( "m Sat Max", colorWindowMagenta, &magenta.sat.max, 255, &configurator::cbUpdateMagentaTrackbar, this );
	createTrackbar( "m Val Min", colorWindowMagenta, &magenta.val.min, 255, &configurator::cbUpdateMagentaTrackbar, this );
	createTrackbar( "m Val Max", colorWindowMagenta, &magenta.val.max, 255, &configurator::cbUpdateMagentaTrackbar, this );
	createTrackbar( "m Erode", colorWindowMagenta, &magenta.erodeSlider, 20, &configurator::cbUpdateMagentaErodeTrackbar, this );
	createTrackbar( "m Dilate", colorWindowMagenta, &magenta.dilateSlider, 20, &configurator::cbUpdateMagentaDilateTrackbar, this );
	createTrackbar( "m Size Min", colorWindowMagenta, &magenta.size, 300);
	createTrackbar( "m Size Max", colorWindowMagenta, &magenta.sizeMax, 2000);

	configWindow = "metrics";
	namedWindow(configWindow, CV_WINDOW_NORMAL);
	createTrackbar( "x  OptBias", configWindow, &xOpticalBias, 120 ); // center is 60, range 0 to 120 = 121 pixels
	createTrackbar( "y  OptBias", configWindow, &yOpticalBias, 720 - viewPixels ); // center is 20, 720-681=39 pixels, range 0 to 39=40 pixels, 39 maps to pixel 0 of 681, so 38+681=719 is available last line
	createTrackbar( "x  Export-100/10", configWindow, &exportOffsetInt.x, 200 ); // center is 100, -100/10 to 100/10 = range -10 pixels to 10 pixels
	createTrackbar( "y  Export-100/10", configWindow, &exportOffsetInt.y, 200 ); // center is 100, -100/10 to 100/10 = range -10 pixels to 10 pixels
	createTrackbar( "rz Export-100/10", configWindow, &exportOffsetInt.rz, 200 ); // center is 100, -100/10 to 100/10 = range -10 degrees to 10 degrees
	createTrackbar( "rz ExpBall-100/10", configWindow, &exportOffsetInt.rzBall, 200 ); // center is 100, -100/10 to 100/10 = range -10 degrees to 10 degrees
	createTrackbar( "rz ExpObst-100/10", configWindow, &exportOffsetInt.rzObstacle, 200 ); // center is 100, -100/10 to 100/10 = range -10 degrees to 10 degrees

	createTrackbar( "x  manual", configWindow, &manual.x, floorWidth-1);
	createTrackbar( "y  manual", configWindow, &manual.y, floorHeight-1);
	createTrackbar( "rz manual", configWindow, &manual.rz, 359);
	createTrackbar( "xStep  solver", configWindow, &solver.xStep, 300);
	createTrackbar( "yStep  solver", configWindow, &solver.yStep, 300);
	createTrackbar( "rzStep solver", configWindow, &solver.rzStep, 200);
	createTrackbar( "maxCnt solver", configWindow, &solver.maxCount, 400);
	// createTrackbar( "score manual", configWindow, &solver.score, 50);
	createTrackbar( "blur pixels", configWindow, &blurPixels, 100);

	cameraWindow = "camera";
	namedWindow(cameraWindow, CV_WINDOW_NORMAL);
	createTrackbar( "backlightComp", cameraWindow, &camera.backlightCompensation, 1);
	createTrackbar( "brightness", cameraWindow, &camera.brightness, 255);
	createTrackbar( "contrast", cameraWindow, &camera.contrast, 255);
	// createTrackbar( "exposureAuto", cameraWindow, &camera.exposureAuto, 1);
	createTrackbar( "exposure", cameraWindow, &camera.exposureAbsolute, 2047);
	// createTrackbar( "exposurePrio", cameraWindow, &camera.exposureAutoPriority, 1);
	createTrackbar( "focusAbs", cameraWindow, &camera.focusAbsolute, 255);
	// createTrackbar( "focusAuto", cameraWindow, &camera.focusAuto, 1);
	createTrackbar( "gain", cameraWindow, &camera.gain, 255);
	// library on robot does not support led createTrackbar( "led1Frequency", cameraWindow, &camera.led1Frequency, 255);
	// library on robot does not support led createTrackbar( "led1Mode", cameraWindow, &camera.led1Mode, 3);
	// createTrackbar( "panAbsolute", cameraWindow, &camera.panAbsolute, 200);
	// createTrackbar( "powerLineFreq", cameraWindow, &camera.powerLineFrequency, 2);
	createTrackbar( "saturation", cameraWindow, &camera.saturation, 255);
	createTrackbar( "sharpness", cameraWindow, &camera.sharpness, 255);
	// createTrackbar( "tiltAbsolute", cameraWindow, &camera.tiltAbsolute, 200);
	createTrackbar( "whiteBalAuto", cameraWindow, &camera.whiteBalanceAuto, 1);
	createTrackbar( "whiteBalance", cameraWindow, &camera.whiteBalanceTemperature, 6500);
	// createTrackbar( "zoom", cameraWindow, &camera.zoomAbsolute, 5);
	createTrackbar( "timingOffset", cameraWindow, &camera.latencyOffset, 1000);

}

void configurator::update( ) {
	// if needed, update the camera with the new values from the Trackbars
	camCtrl.update( camera, true );
}

void configurator::updateLineTrackbar( int ) {
	if( lineSt.sat.max < lineSt.sat.min ) { lineSt.sat.max = lineSt.sat.min; setTrackbarPos( "l Sat Max", colorWindowLine, lineSt.sat.max ); }
	if( lineSt.sat.min > lineSt.sat.max ) { lineSt.sat.min = lineSt.sat.max; setTrackbarPos( "l Sat Min", colorWindowLine, lineSt.sat.min ); }
	if( lineSt.val.max < lineSt.val.min ) { lineSt.val.max = lineSt.val.min; setTrackbarPos( "l Val Max", colorWindowLine, lineSt.val.max ); }
	if( lineSt.val.min > lineSt.val.max ) { lineSt.val.min = lineSt.val.max; setTrackbarPos( "l Val Min", colorWindowLine, lineSt.val.min ); }
	drawLineBGR( );
}
void configurator::updateFloorTrackbar( int ) {
	if( floorSt.sat.max < floorSt.sat.min ) { floorSt.sat.max = floorSt.sat.min; setTrackbarPos( "f Sat Max", colorWindowFloor, floorSt.sat.max ); }
	if( floorSt.sat.min > floorSt.sat.max ) { floorSt.sat.min = floorSt.sat.max; setTrackbarPos( "f Sat Min", colorWindowFloor, floorSt.sat.min ); }
	if( floorSt.val.max < floorSt.val.min ) { floorSt.val.max = floorSt.val.min; setTrackbarPos( "f Val Max", colorWindowFloor, floorSt.val.max ); }
	if( floorSt.val.min > floorSt.val.max ) { floorSt.val.min = floorSt.val.max; setTrackbarPos( "f Val Min", colorWindowFloor, floorSt.val.min ); }
	drawFloorBGR( );
}
void configurator::updateBallTrackbar( int ) {
	if( ball.sat.max < ball.sat.min ) { ball.sat.max = ball.sat.min; setTrackbarPos( "b Sat Max", colorWindowBall, ball.sat.max ); }
	if( ball.sat.min > ball.sat.max ) { ball.sat.min = ball.sat.max; setTrackbarPos( "b Sat Min", colorWindowBall, ball.sat.min ); }
	if( ball.val.max < ball.val.min ) { ball.val.max = ball.val.min; setTrackbarPos( "b Val Max", colorWindowBall, ball.val.max ); }
	if( ball.val.min > ball.val.max ) { ball.val.min = ball.val.max; setTrackbarPos( "b Val Min", colorWindowBall, ball.val.min ); }
	drawBallBGR( );
}
void configurator::updatePossessionTrackbar( int ) {
	if( possession.sat.max < possession.sat.min ) { possession.sat.max = possession.sat.min; setTrackbarPos( "p Sat Max", colorWindowPossession, possession.sat.max ); }
	if( possession.sat.min > possession.sat.max ) { possession.sat.min = possession.sat.max; setTrackbarPos( "p Sat Min", colorWindowPossession, possession.sat.min ); }
	if( possession.val.max < possession.val.min ) { possession.val.max = possession.val.min; setTrackbarPos( "p Val Max", colorWindowPossession, possession.val.max ); }
	if( possession.val.min > possession.val.max ) { possession.val.min = possession.val.max; setTrackbarPos( "p Val Min", colorWindowPossession, possession.val.min ); }
	possession.hue.min = possession.hue.center - (possession.hue.delta-1)/2; // -1 for slider granularity
	possession.hue.max = possession.hue.center + possession.hue.delta/2;
}
void configurator::updateObstacleTrackbar( int ) {
	if( obstacle.sat.max < obstacle.sat.min ) { obstacle.sat.max = obstacle.sat.min; setTrackbarPos( "o Sat Max", colorWindowObstacle, obstacle.sat.max ); }
	if( obstacle.sat.min > obstacle.sat.max ) { obstacle.sat.min = obstacle.sat.max; setTrackbarPos( "o Sat Min", colorWindowObstacle, obstacle.sat.min ); }
	if( obstacle.val.max < obstacle.val.min ) { obstacle.val.max = obstacle.val.min; setTrackbarPos( "o Val Max", colorWindowObstacle, obstacle.val.max ); }
	if( obstacle.val.min > obstacle.val.max ) { obstacle.val.min = obstacle.val.max; setTrackbarPos( "o Val Min", colorWindowObstacle, obstacle.val.min ); }
	drawObstacleBGR( );
}
void configurator::updateCyanTrackbar( int ) {
	if( cyan.sat.max < cyan.sat.min ) { cyan.sat.max = cyan.sat.min; setTrackbarPos( "c Sat Max", colorWindowCyan, cyan.sat.max ); }
	if( cyan.sat.min > cyan.sat.max ) { cyan.sat.min = cyan.sat.max; setTrackbarPos( "c Sat Min", colorWindowCyan, cyan.sat.min ); }
	if( cyan.val.max < cyan.val.min ) { cyan.val.max = cyan.val.min; setTrackbarPos( "c Val Max", colorWindowCyan, cyan.val.max ); }
	if( cyan.val.min > cyan.val.max ) { cyan.val.min = cyan.val.max; setTrackbarPos( "c Val Min", colorWindowCyan, cyan.val.min ); }
	drawCyanBGR( );
}
void configurator::updateMagentaTrackbar( int ) {
	if( magenta.sat.max < magenta.sat.min ) { magenta.sat.max = magenta.sat.min; setTrackbarPos( "m Sat Max", colorWindowMagenta, magenta.sat.max ); }
	if( magenta.sat.min > magenta.sat.max ) { magenta.sat.min = magenta.sat.max; setTrackbarPos( "m Sat Min", colorWindowMagenta, magenta.sat.min ); }
	if( magenta.val.max < magenta.val.min ) { magenta.val.max = magenta.val.min; setTrackbarPos( "m Val Max", colorWindowMagenta, magenta.val.max ); }
	if( magenta.val.min > magenta.val.max ) { magenta.val.min = magenta.val.max; setTrackbarPos( "m Val Min", colorWindowMagenta, magenta.val.min ); }
	drawMagentaBGR( );
}

// create color window for white lines
void configurator::drawLineBGR( ) {
	lineSt.hue.min = lineSt.hue.center - (lineSt.hue.delta-1)/2; // -1 for slider granularity
	lineSt.hue.max = lineSt.hue.center + lineSt.hue.delta/2;
   	Mat lineStHSV = Mat::zeros(60, 150, CV_8UC3);
   	for( int col = 0 ; col < lineStHSV.cols; col++ ) {
   		int hue = lineSt.hue.min + (int)(0.5f + (1.f*col*(lineSt.hue.max-lineSt.hue.min))/(lineStHSV.cols-1));
   		if( hue < 0 ) { hue = hue + 180; }
   		if( hue > 179 ) { hue = hue - 180; }
   		for( int row = 0; row < lineStHSV.rows; row++ ) {
   			int sat = lineSt.sat.min + (int)(0.5f + (1.f*row*(lineSt.sat.max-lineSt.sat.min))/(lineStHSV.rows-1)); // Saturation
   			line( lineStHSV, Point(col,row), Point(col,row), Scalar(hue,sat,lineSt.val.max), 1);
   		}
   	}
	cvtColor(lineStHSV, lineSt.window, COLOR_HSV2BGR); // convert to BGR
}

// create color window for floor
void configurator::drawFloorBGR( ) {
	floorSt.hue.min = floorSt.hue.center - (floorSt.hue.delta-1)/2; // -1 for slider granularity
	floorSt.hue.max = floorSt.hue.center + floorSt.hue.delta/2;
   	Mat floorStHSV = Mat::zeros(60, 150, CV_8UC3);
   	for( int col = 0 ; col < floorStHSV.cols; col++ ) {
   		int hue = floorSt.hue.min + (int)(0.5f + (1.f*col*(floorSt.hue.max-floorSt.hue.min))/(floorStHSV.cols-1));
   		if( hue < 0 ) { hue = hue + 180; }
   		if( hue > 179 ) { hue = hue - 180; }
   		for( int row = 0; row < floorStHSV.rows; row++ ) {
   			int sat = floorSt.sat.min + (int)(0.5f + (1.f*row*(floorSt.sat.max-floorSt.sat.min))/(floorStHSV.rows-1)); // Saturation
   			line( floorStHSV, Point(col,row), Point(col,row), Scalar(hue,sat,floorSt.val.max), 1);
   		}
   	}
	cvtColor(floorStHSV, floorSt.window, COLOR_HSV2BGR); // convert to BGR
}

// create color window for yellow ball
void configurator::drawBallBGR( ) {
	ball.hue.min = ball.hue.center - (ball.hue.delta-1)/2; // -1 for slider granularity
	ball.hue.max = ball.hue.center + ball.hue.delta/2;
   	Mat ballHSV = Mat::zeros(60, 150, CV_8UC3);
   	for( int col = 0 ; col < ballHSV.cols; col++ ) {
   		int hue = ball.hue.min + (int)(0.5f + (1.f*col*(ball.hue.max-ball.hue.min))/(ballHSV.cols-1));
   		if( hue < 0 ) { hue = hue + 180; }
   		if( hue > 179 ) { hue = hue - 180; }
   		for( int row = 0; row < ballHSV.rows; row++ ) {
   			int sat = ball.sat.min + (int)(0.5f + (1.f*row*(ball.sat.max-ball.sat.min))/(ballHSV.rows-1)); // Saturation
   			line( ballHSV, Point(col,row), Point(col,row), Scalar(hue,sat,ball.val.max), 1);
   		}
   	}
	cvtColor(ballHSV, ball.window, COLOR_HSV2BGR); // convert to BGR
}

// create color window for interfering stuff, e.g. light reflections
void configurator::drawObstacleBGR( ) {
	obstacle.hue.min = obstacle.hue.center - (obstacle.hue.delta-1)/2; // -1 for slider granularity
	obstacle.hue.max = obstacle.hue.center + obstacle.hue.delta/2;
   	Mat obstacleHSV = Mat::zeros(60, 150, CV_8UC3);
   	for( int col = 0 ; col < obstacleHSV.cols; col++ ) {
   		int hue = obstacle.hue.min + (int)(0.5f + (1.f*col*(obstacle.hue.max-obstacle.hue.min))/(obstacleHSV.cols-1));
   		if( hue < 0 ) { hue = hue + 180; }
   		if( hue > 179 ) { hue = hue - 180; }
   		for( int row = 0; row < obstacleHSV.rows; row++ ) {
   			int sat = obstacle.sat.min + (int)(0.5f + (1.f*row*(obstacle.sat.max-obstacle.sat.min))/(obstacleHSV.rows-1)); // Saturation
   			line( obstacleHSV, Point(col,row), Point(col,row), Scalar(hue,sat,obstacle.val.max), 1);
   		}
   	}
	cvtColor(obstacleHSV, obstacle.window, COLOR_HSV2BGR); // convert to BGR
}

// create color window for the cyan robot mark
void configurator::drawCyanBGR( ) {
	cyan.hue.min = cyan.hue.center - (cyan.hue.delta-1)/2; // -1 for slider granularity
	cyan.hue.max = cyan.hue.center + cyan.hue.delta/2;
   	Mat cyanHSV = Mat::zeros(60, 150, CV_8UC3);
   	for( int col = 0 ; col < cyanHSV.cols; col++ ) {
   		int hue = cyan.hue.min + (int)(0.5f + (1.f*col*(cyan.hue.max-cyan.hue.min))/(cyanHSV.cols-1));
   		if( hue < 0 ) { hue = hue + 180; }
   		if( hue > 179 ) { hue = hue - 180; }
   		for( int row = 0; row < cyanHSV.rows; row++ ) {
   			int sat = cyan.sat.min + (int)(0.5f + (1.f*row*(cyan.sat.max-cyan.sat.min))/(cyanHSV.rows-1)); // Saturation
   			line( cyanHSV, Point(col,row), Point(col,row), Scalar(hue,sat,cyan.val.max), 1);
   		}
   	}
	cvtColor(cyanHSV, cyan.window, COLOR_HSV2BGR); // convert to BGR
}

// create color window for the cyan robot mark
void configurator::drawMagentaBGR( ) {
	magenta.hue.min = magenta.hue.center - (magenta.hue.delta-1)/2; // -1 for slider granularity
	magenta.hue.max = magenta.hue.center + magenta.hue.delta/2;
   	Mat magentaHSV = Mat::zeros(60, 150, CV_8UC3);
   	for( int col = 0 ; col < magentaHSV.cols; col++ ) {
   		int hue = magenta.hue.min + (int)(0.5f + (1.f*col*(magenta.hue.max-magenta.hue.min))/(magentaHSV.cols-1));
   		if( hue < 0 ) { hue = hue + 180; }
   		if( hue > 179 ) { hue = hue - 180; }
   		for( int row = 0; row < magentaHSV.rows; row++ ) {
   			int sat = magenta.sat.min + (int)(0.5f + (1.f*row*(magenta.sat.max-magenta.sat.min))/(magentaHSV.rows-1)); // Saturation
   			line( magentaHSV, Point(col,row), Point(col,row), Scalar(hue,sat,magenta.val.max), 1);
   		}
   	}
	cvtColor(magentaHSV, magenta.window, COLOR_HSV2BGR); // convert to BGR
}


// convert pixels from the distorted round viewer to rectangular soccer floor
double configurator::calcDeWarp( double pixel ) {
	return deWarp.outSideTan * tan ( pixel * deWarp.inSideTan ) + pixel * deWarp.pixelPow1 + pow(pixel,2) * deWarp.pixelPow2;
}

colStruct configurator::getBall( size_t type ) {
	if( type == ballType ) {
		return ball;
	} else if ( type == cyanType ) {
		return cyan;
	} else if ( type == magentaType ) {
		return magenta;
	} else {
		printf("ERROR     : type for getBall undefined %zu\n", type);
		exit( EXIT_FAILURE );
	}
}
