// Copyright 2018 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2016-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef CAMERA_CONTROL_HPP
#define CAMERA_CONTROL_HPP

#include <linux/videodev2.h>

typedef struct {
	int backlightCompensation; // bool
	int brightness;
	int contrast;
	int exposureAbsolute;
	int exposureAuto; // bool
	int exposureAutoPriority; // bool
	int focusAbsolute;
	int focusAuto;
	int gain;
	int led1Frequency;
	int led1Mode;
	int panAbsolute;
	int powerLineFrequency;
	int saturation;
	int sharpness;
	int tiltAbsolute;
	int whiteBalanceAuto; // bool
	int whiteBalanceTemperature;
	int zoomAbsolute;
	int latencyOffset;
} cameraSt;

class cameraControl
{
private:
	int cameraHandle; // file descriptor to camera, set once through initialize
	struct v4l2_control control; // used for ioctl
	size_t reInitAgainCounter;

	int deviceIndexUsed; // contains the /dev/videoX index for the Logitech HD Webcam C525 or -1 if not available
	size_t gainForceCounter;

	cameraSt configCache; // use cache for configuration values to minimize ioctl's to camera

	void verboseCameraSize( ); // verbose configured width and height of the camera
	void getSettings( ); // verbose configuration settings from camera
	void getCamera( ); // search for Logitech HD Webcam C525
	void clearConfigCache( ); // used to force writing the camera settings again

	std::string idToDescription( int id );
	int get( int id, bool verbose ); // ioctl getter
	void set( int id, int value, bool verbose ); // ioctl setter

public:
	cameraControl( );
	int getVideoDevice( ) { return deviceIndexUsed; } // get index for /dev/videoX of Logitech HD Webcam C525
	void update( cameraSt camera, bool verbose ); // if needed update camera settings like brightness
};

#endif
