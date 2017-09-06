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

// Class to configure the frontVision system
// e.g. set ball color or configure camera

#include <unistd.h> // for getuid()
#include <pwd.h> // for getpwuid()
#include <stdio.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "configurator.hpp"

using namespace std;
using namespace cv;

configurator::configurator( ) {
	char configFile[256];
	sprintf( configFile, "%s/falcons/code/packages/frontVision/frontVision.yaml", getpwuid(getuid())->pw_dir );
	FileStorage fs(configFile, FileStorage::READ);

	FileNode ballFn = fs["ball"];
	ball.hueCenter = (int)(ballFn["hueCenter"]);
	ball.hueDelta = (int)(ballFn["hueDelta"]);
	ball.satMin = (int)(ballFn["satMin"]);
	ball.satMax = (int)(ballFn["satMax"]);
	ball.valMin = (int)(ballFn["valMin"]);
	ball.valMax = (int)(ballFn["valMax"]);
	ball.minimalSize = (int)(ballFn["minimalSize"]);

	FileNode cameraFn = fs["camera"];
	camera.backlightCompensation = (int)(cameraFn["backlightCompensation"]);
	camera.brightness = (int)(cameraFn["brightness"]);
	camera.contrast = (int)(cameraFn["contrast"]);
	camera.exposureAbsolute = (int)(cameraFn["exposureAbsolute"]);
	camera.exposureAuto = (int)(cameraFn["exposureAuto"]);
	camera.gamma = (int)(cameraFn["gamma"]);
	camera.hue = (int)(cameraFn["hue"]);
	camera.powerLineFrequency = (int)(cameraFn["powerLineFrequency"]);
	camera.saturation = (int)(cameraFn["saturation"]);
	camera.sharpness = (int)(cameraFn["sharpness"]);
	camera.whiteBalanceTemperature = (int)(cameraFn["whiteBalanceTemperature"]);
	camera.whiteBalanceAuto = (int)(cameraFn["whiteBalanceAuto"]);

	fs.release( );

	configWindows( );
}

void configurator::configWindows( ) {
	std::string ballWindow = "front vision ball";
	namedWindow(ballWindow, CV_WINDOW_NORMAL);
	createTrackbar( "Hue Cent", ballWindow, &ball.hueCenter, 179); // hue range is 0 to 179
	createTrackbar( "Hue Delta", ballWindow, &ball.hueDelta, 180);
	createTrackbar( "Sat Min", ballWindow, &ball.satMin, 255);
	createTrackbar( "Sat Max", ballWindow, &ball.satMax, 255);
	createTrackbar( "Val Min", ballWindow, &ball.valMin, 255);
	createTrackbar( "Val Max", ballWindow, &ball.valMax, 255);
	createTrackbar( "Min Size", ballWindow, &ball.minimalSize, 1000);

	std::string camWindow = "front vision camera";
	namedWindow(camWindow, CV_WINDOW_NORMAL);
	createTrackbar( "Backlight", camWindow, &camera.backlightCompensation, 1 ); // (int) : min=0 max=1 step=1 default=0 value=0
	createTrackbar( "Brightness", camWindow, &camera.brightness, 64+64); // (int) : min=-64 max=64 step=1 default=0 value=0
	createTrackbar( "Contrast", camWindow, &camera.contrast, 95); // (int) : min=0 max=95 step=1 default=32 value=32
	// very high exposure values slow down the fps significantly, 1000 already drop to 14.6 fps, upto 600 30 fps can be achieved
	createTrackbar( "Exposure", camWindow, &camera.exposureAbsolute, 1000); // (int) : min=50 max=10000 step=1 default=166 value=166 flags=inactive
	createTrackbar( "Exposure Auto", camWindow, &camera.exposureAuto, 3-2); // (menu) 1: Manual Mode, 3: Aperture Priority Mode, use this as 0 manual and 1 auto !
	createTrackbar( "Gamma", camWindow, &camera.gamma, 300); // (int) : min=100 max=300 step=1 default=165 value=165
	createTrackbar( "Hue", camWindow, &camera.hue, 2000+2000); // (int) : min=-2000 max=2000 step=1 default=0 value=0
	createTrackbar( "PowerLineFreq", camWindow, &camera.powerLineFrequency, 2); // (menu) 0: Disabled, 1: 50 Hz, 2: 60 Hz
	createTrackbar( "Saturation", camWindow, &camera.saturation, 100); // (int) : min=0 max=100 step=1 default=55 value=55
	createTrackbar( "Sharpness", camWindow, &camera.sharpness, 7); // (int) : min=1 max=7 step=1 default=2 value=2
	createTrackbar( "WhiteBalance", camWindow, &camera.whiteBalanceTemperature, 6500); // (int) : min=2800 max=6500 step=10 default=4600 value=4600 flags=inactive
	createTrackbar( "WhiteBal Auto", camWindow, &camera.whiteBalanceAuto, 1); // (bool) : default=1 value=1

}

void configurator::update( ) {

	// if needed, update the camera with the new values from the Trackbars
	camCtrl.update( camera, true );
}
