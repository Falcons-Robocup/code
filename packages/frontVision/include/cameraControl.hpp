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

#ifndef CAMERA_CONTROL_HPP
#define CAMERA_CONTROL_HPP

#include <linux/videodev2.h>

typedef struct {
	int backlightCompensation; // bool
	int brightness;
	int contrast;
	int exposureAbsolute;
	int exposureAuto;
	int gamma;
	int hue;
	int powerLineFrequency;
	int saturation;
	int sharpness;
	int whiteBalanceTemperature;
	int whiteBalanceAuto; // bool
} cameraSt;

class cameraControl
{
private:
	int cameraHandle; // file descriptor to camera, set once through initialize
	struct v4l2_control control; // used for ioctl

	int deviceIndexUsed; // contains the /dev/videoX index for the Genius WideCam F100 or -1 if not available

	cameraSt configCache; // use cache for configuration values to minimize ioctl's to camera

	void verboseCameraSize( ); // verbose configured width and height of the camera
	void getSettings( ); // verbose configuration settings from camera
	void getCamera( ); // search for Genius WideCam F100

	std::string idToDescription( int id );
	int get( int id, bool verbose ); // ioctl getter
	void set( int id, int value, bool verbose ); // ioctl setter

public:
	cameraControl( );
	int getVideoDevice( ) { return deviceIndexUsed; } // get index for /dev/videoX of Genius WideCam F100
	void update( cameraSt camera, bool verbose ); // if needed update camera settings like brightness
};

#endif
