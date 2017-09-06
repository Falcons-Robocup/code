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

// Camera control is used to find the Genius WideCam F100 on one of the /dev/videoX devices
// and it is also used to control the camera settings like brightness.

/* Genius WideCam F100 capabilities (from v4l2-ctl -L)

                     brightness (int)    : min=-64 max=64 step=1 default=0 value=0
                       contrast (int)    : min=0 max=95 step=1 default=32 value=32
              exposure_absolute (int)    : min=50 max=10000 step=1 default=166 value=166 flags=inactive
                  exposure_auto (menu)   : min=0 max=3 default=3 value=3
                                1: Manual Mode
                                3: Aperture Priority Mode
                          gamma (int)    : min=100 max=300 step=1 default=165 value=165
                            hue (int)    : min=-2000 max=2000 step=1 default=0 value=0
           power_line_frequency (menu)   : min=0 max=2 default=1 value=1
                                0: Disabled
                                1: 50 Hz
                                2: 60 Hz
                      sharpness (int)    : min=1 max=7 step=1 default=2 value=2
         backlight_compensation (int)    : min=0 max=1 step=1 default=0 value=0
                     saturation (int)    : min=0 max=100 step=1 default=55 value=55
 white_balance_temperature_auto (bool)   : default=1 value=1
*/

#include <cerrno>
#include <dirent.h> // for DIR and struct dirent
#include <stdio.h>
#include <sys/ioctl.h>
#include <iostream>
#include <fcntl.h> // for O_RDWR
#include <unistd.h>


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "cameraControl.hpp"

using namespace std;
using namespace cv;

cameraControl::cameraControl( ) {
	// be sure the config values are overwritten from the configuration by setting to not used value
	configCache.backlightCompensation = -1;
	configCache.brightness = -1;
	configCache.contrast = -1;
	configCache.exposureAbsolute = -1;
	configCache.exposureAuto = -1;
	configCache.gamma = -1;
	configCache.hue = -1;
	configCache.powerLineFrequency = -1;
	configCache.saturation = -1;
	configCache.sharpness = -1;
	configCache.whiteBalanceTemperature = -1;
	configCache.whiteBalanceAuto = -1;

	deviceIndexUsed = -1; // -1 means no camera found

	getCamera( ); // search for the Genius WideCam F100

	// TODO: call after initialization, or set resolution also on this level
	verboseCameraSize( );

	// printf("INFO    : current values camera:\n");
	// getSettings( );
	// printf("\n");
}

void cameraControl::getCamera( ) {
	char device[32];
	DIR *dp;
	struct dirent *ep;
	std::vector<int> deviceList;

	dp = opendir("/dev");
	if( dp == NULL ) {
		fprintf( stderr, "ERROR   : %s when opening /dev for reading\n", strerror(errno) );
		exit( EXIT_FAILURE );
	}
	while( ( ep = readdir(dp) ) ) {
		if( memcmp( ep->d_name, "video", 5 ) == 0 ) { // compare first 5 characters of ep->d_name
			char stringIndex[4] = "";
			strncpy( stringIndex, ep->d_name +5, 3 ); // unlikely there will be more then 99 video devices (and \n)
			int deviceIndex = atoi(stringIndex);
			// printf( "INFO    : found device /dev/video%d\n", deviceIndex );
			deviceList.push_back(deviceIndex); // create list of all video devices
		}
	}
	closedir( dp );

	sort(deviceList.begin(), deviceList.end()); // order the /dev/video0 to /dev/videoX

	for( size_t ii = 0; ii < deviceList.size(); ii++ ) {
		sprintf(device, "/dev/video%d", deviceList[ii] );
		// printf( "INFO    : found device %s\n", device );

		cameraHandle = open( device, O_RDWR );
		if( cameraHandle == -1 || cameraHandle == 0 ) {
			fprintf( stderr, "ERROR   : %s when opening capture device %s\n", strerror(errno), device );
			exit( EXIT_FAILURE );
		}

		// get camera capabilities
		struct v4l2_capability caps;
		if( ioctl( cameraHandle, VIDIOC_QUERYCAP, &caps ) == -1 ) {
			fprintf( stderr, "ERROR   : %s when query capabilities of %s \n", strerror(errno), device );
			exit( EXIT_FAILURE );
		}
		close( cameraHandle );

		// show camera capabilities
		// not used values: Linux kernel version, caps.driver, caps.reserved and physical device capabilities
		printf( "INFO    : device %s name %-32s bus %-24s capabilities 0x%08x\n",
				device, caps.card, caps.bus_info, caps.device_caps );

		// the Genius WideCam F100 uses name "USB_Camera", which is pretty generic, but different from the other used camera's
		if( memcmp( caps.card, "USB_Camera", 10 ) == 0 ) {
			deviceIndexUsed = deviceList[ii];
		}
	}

	if( deviceIndexUsed == -1 ) {
		fprintf( stderr, "ERROR   : no Genius WideCam F100 has been found\n" );
		exit( EXIT_FAILURE );
	} else {
		sprintf(device, "/dev/video%d", deviceIndexUsed );
		printf( "INFO    : the Genius WideCam F100 can be accessed through %s\n", device );
		cameraHandle = open( device, O_RDWR );
	}
}

string cameraControl::idToDescription( int id ) {
	if( id == V4L2_CID_BACKLIGHT_COMPENSATION ) { return "backlight compensation"; }
	else if( id == V4L2_CID_BRIGHTNESS ) { return "brightness"; }
	else if( id == V4L2_CID_CONTRAST ) { return  "contrast"; }
	else if( id == V4L2_CID_EXPOSURE_AUTO ) { return  "exposure auto"; }
	else if( id == V4L2_CID_EXPOSURE_ABSOLUTE ) { return "exposure"; }
	else if( id == V4L2_CID_POWER_LINE_FREQUENCY ) { return "power line frequency"; }
	else if( id == V4L2_CID_SATURATION ) { return "saturation"; }
	else if( id == V4L2_CID_SHARPNESS ) { return "sharpness"; }
	else if( id == V4L2_CID_AUTO_WHITE_BALANCE ) { return "white balance auto"; }
	else if( id == V4L2_CID_WHITE_BALANCE_TEMPERATURE ) { return "white balance"; }
	else if( id == V4L2_CID_GAMMA ) { return "gamma"; }
	else if( id == V4L2_CID_HUE ) { return "hue"; }
	else { return "unknown ID"; }
}


// get width and height from the camera
void cameraControl::verboseCameraSize( ) {
	struct v4l2_format vfmt;
	memset(&vfmt, 0, sizeof(vfmt));

	vfmt.fmt.pix.priv = 0xfeedcafe; // the v4l2 on the robots does not have V4L2_PIX_FMT_PRIV_MAGIC
	vfmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if( ioctl(cameraHandle, VIDIOC_G_FMT, &vfmt) == -1 ) {
		fprintf( stderr, "ERROR   : %s when trying to get pixel format\n", strerror(errno) );
		exit( EXIT_FAILURE );
	}
	printf( "INFO    : width %u height %u size %u\n", vfmt.fmt.pix.width, vfmt.fmt.pix.height, vfmt.fmt.pix.sizeimage);
}

// get settings value from camera
int cameraControl::get( int id, bool verbose ) {
	control.id = (__u32) id;
	control.value = 0;

	if( ioctl(cameraHandle, VIDIOC_G_CTRL, &control ) == -1 ) {
		fprintf( stderr, "ERROR   : %s when trying to get camera %s value\n", strerror(errno), idToDescription(id).c_str() );
		exit( EXIT_FAILURE );
	}
	if( verbose ) {
		printf( "INFO    : %s value is %d\n", idToDescription(id).c_str(), control.value);
	}
	return (int) control.value;
}

// write setting value to camera
void cameraControl::set( int id, int value, bool verbose ) {
	control.id = (__u32) id;
	control.value = (__s32) value;

	if( -1 == ioctl(cameraHandle, VIDIOC_S_CTRL, &control) ) {
		fprintf( stderr, "ERROR   : %s when trying to set camera %s to value %d\n", strerror(errno), idToDescription(id).c_str(), value );
		exit( EXIT_FAILURE );
	}
	if( verbose ) {
		printf( "INFO    : %s set to %d\n", idToDescription(id).c_str(), control.value);
	}
}

void cameraControl::getSettings( ) {
	bool verbose = true;
	get( V4L2_CID_BACKLIGHT_COMPENSATION, verbose );
	get( V4L2_CID_BRIGHTNESS, verbose );
	get( V4L2_CID_CONTRAST, verbose );
	get( V4L2_CID_EXPOSURE_ABSOLUTE, verbose );
	get( V4L2_CID_EXPOSURE_AUTO, verbose );
	get( V4L2_CID_GAMMA, verbose );
	get( V4L2_CID_HUE, verbose );
	get( V4L2_CID_POWER_LINE_FREQUENCY, verbose );
	get( V4L2_CID_SATURATION, verbose );
	get( V4L2_CID_SHARPNESS, verbose );
	get( V4L2_CID_WHITE_BALANCE_TEMPERATURE, verbose );
	get( V4L2_CID_AUTO_WHITE_BALANCE, verbose );
}

void cameraControl::update( cameraSt camera, bool verbose ) {

	// check if the camera still works (e.g. because of USB reset or reconnect) by reading one of the values
	get( V4L2_CID_BRIGHTNESS, false );

	if( configCache.backlightCompensation != camera.backlightCompensation ) {
		set( V4L2_CID_BACKLIGHT_COMPENSATION, camera.backlightCompensation, verbose );
		configCache.backlightCompensation = camera.backlightCompensation;
	}

	if( configCache.brightness != camera.brightness ) {
		set( V4L2_CID_BRIGHTNESS, camera.brightness - 64, verbose );
		configCache.brightness = camera.brightness;
	}

	if( configCache.contrast != camera.contrast ) {
		set( V4L2_CID_CONTRAST, camera.contrast, verbose );
		configCache.contrast = camera.contrast;
	}

	if( configCache.exposureAuto != camera.exposureAuto ) {
		if( camera.exposureAuto == 1 ) {
	        // Aperture Priority Mode
			set( V4L2_CID_EXPOSURE_AUTO, 3, verbose );
		} else {
	        // Manual Mode
			set( V4L2_CID_EXPOSURE_AUTO, 1, verbose );
			set( V4L2_CID_EXPOSURE_ABSOLUTE, camera.exposureAbsolute, verbose );
		}
		configCache.exposureAuto = camera.exposureAuto;
	}

	if( configCache.exposureAbsolute != camera.exposureAbsolute ) {
		if( camera.exposureAuto != 1 ) {
	        // Manual Mode
			if( camera.exposureAbsolute < 50 ) {
				set( V4L2_CID_EXPOSURE_ABSOLUTE, 50, verbose );
			} else {
				set( V4L2_CID_EXPOSURE_ABSOLUTE, camera.exposureAbsolute, verbose );
			}
		}
		configCache.exposureAbsolute = camera.exposureAbsolute;
	}

	if( configCache.gamma != camera.gamma ) {
		if( camera.gamma < 100 ) {
			set( V4L2_CID_GAMMA, 100, verbose );
		} else {
			set( V4L2_CID_GAMMA, camera.gamma, verbose );
		}
		configCache.gamma = camera.gamma;
	}

	if( configCache.hue != camera.hue ) {
		set( V4L2_CID_HUE, camera.hue - 2000, verbose );
		configCache.hue = camera.hue;
	}

	if( configCache.powerLineFrequency != camera.powerLineFrequency ) {
		set( V4L2_CID_POWER_LINE_FREQUENCY, camera.powerLineFrequency, verbose ); // this one after the exposure otherwise flickering video
		configCache.powerLineFrequency = camera.powerLineFrequency;
	}

	if( configCache.saturation != camera.saturation ) {
		set( V4L2_CID_SATURATION, camera.saturation, verbose );
		configCache.saturation = camera.saturation;
	}

	if( configCache.sharpness != camera.sharpness ) {
		if( camera.sharpness < 1 ) {
			set( V4L2_CID_SHARPNESS, 1, verbose );

		} else {
			set( V4L2_CID_SHARPNESS, camera.sharpness, verbose );
		}
		configCache.sharpness = camera.sharpness;
	}

	if( configCache.whiteBalanceAuto != camera.whiteBalanceAuto ) {
		set( V4L2_CID_AUTO_WHITE_BALANCE, camera.whiteBalanceAuto, verbose );
		if( camera.whiteBalanceAuto != 1 ) {
			if( camera.whiteBalanceTemperature < 2800 ) {
				set( V4L2_CID_WHITE_BALANCE_TEMPERATURE, 2800, verbose );
			} else {
				set( V4L2_CID_WHITE_BALANCE_TEMPERATURE, camera.whiteBalanceTemperature, verbose );
			}
		}
		configCache.whiteBalanceAuto = camera.whiteBalanceAuto;
	}

	if( configCache.whiteBalanceTemperature != camera.whiteBalanceTemperature ) {
		if( camera.whiteBalanceAuto != 1 ) {
			if( camera.whiteBalanceTemperature < 2800 ) {
				set( V4L2_CID_WHITE_BALANCE_TEMPERATURE, 2800, verbose );
			} else {
				set( V4L2_CID_WHITE_BALANCE_TEMPERATURE, camera.whiteBalanceTemperature, verbose );
			}
		}
		configCache.whiteBalanceTemperature = camera.whiteBalanceTemperature;
	}
}
