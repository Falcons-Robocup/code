 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2016-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// Camera control is used to find the Genius WideCam F100 on one of the /dev/videoX devices
// and it is also used to control the camera settings like brightness.

/* Logitech HD Webcam C525 (from v4l2-ctl -L)

         backlight_compensation (int)    : min=0 max=1 step=1 default=1
                     brightness (int)    : min=0 max=255 step=1 default=128
                       contrast (int)    : min=0 max=255 step=1 default=32
              exposure_absolute (int)    : min=3 max=2047 step=1 default=166
                  exposure_auto (menu)   : min=0 max=3 default=3
				1: Manual Mode
				3: Aperture Priority Mode
         exposure_auto_priority (bool)   : default=0 value=1
                 focus_absolute (int)    : min=0 max=255 step=5 default=60
                     focus_auto (bool)   : default=1
                           gain (int)    : min=0 max=255 step=1 default=64
                 led1_frequency (int)    : min=0 max=255 step=1 default=0
                      led1_mode (menu)   : min=0 max=3 default=0
				0: Off
				1: On
				2: Blink
				3: Auto
                   pan_absolute (int)    : min=-36000 max=36000 step=3600 default=0
           power_line_frequency (menu)   : min=0 max=2 default=2
				0: Disabled
				1: 50 Hz
				2: 60 Hz
                     saturation (int)    : min=0 max=255 step=1 default=32
                      sharpness (int)    : min=0 max=255 step=1 default=22
                  tilt_absolute (int)    : min=-36000 max=36000 step=3600 default=0
 white_balance_temperature_auto (bool)   : default=1
      white_balance_temperature (int)    : min=2800 max=6500 step=1 default=5500
                  zoom_absolute (int)    : min=1 max=5 step=1 default=1
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

#define V4L2_CID_LED1_FREQUENCY 0x0A046D06
#define V4L2_CID_LED1_MODE 0x0A046D05

cameraControl::cameraControl( ) {
	// be sure the config values are overwritten from the configuration by setting to not used value
	clearConfigCache();

	deviceIndexUsed = -1; // -1 means no camera found
	gainForceCounter = 0;
	reInitAgainCounter = 0;


	getCamera( ); // search for the Logitech HD Webcam C525

	if( deviceIndexUsed >= 0 ) {
		// TODO: call after initialization, or set resolution also on this level
		verboseCameraSize( );
	}

	// printf("INFO    : current values camera:\n");
	// getSettings( );
	// printf("\n");
}

void cameraControl::clearConfigCache( ) {
	configCache.backlightCompensation = -1;
	configCache.brightness = -1;
	configCache.contrast = -1;
	configCache.exposureAbsolute = -1;
	configCache.exposureAuto = -1;
	configCache.exposureAutoPriority = -1;
	configCache.focusAbsolute = -1;
	configCache.focusAuto = -1;
	configCache.gain = -1;
	configCache.led1Frequency = -1;
	configCache.led1Mode = -1;
	configCache.panAbsolute = -1;
	configCache.powerLineFrequency = -1;
	configCache.saturation = -1;
	configCache.sharpness = -1;
	configCache.tiltAbsolute = -1;
	configCache.whiteBalanceAuto = -1;
	configCache.whiteBalanceTemperature = -1;
	configCache.zoomAbsolute = -1;
}

void cameraControl::getCamera( ) {
	char device[32];
	DIR *dp;
	struct dirent *ep;
	std::vector<int> deviceList;

	dp = opendir("/dev");
	if( dp == NULL ) {
		printf( "ERROR     : %s when opening /dev for reading\n", strerror(errno) );
		exit( EXIT_FAILURE );
	}
	while( ( ep = readdir(dp) ) ) {
		if( memcmp( ep->d_name, "video", 5 ) == 0 ) { // compare first 5 characters of ep->d_name
			char stringIndex[4] = "";
			strncpy( stringIndex, ep->d_name +5, 3 ); // unlikely there will be more then 99 video devices (and \n)
			int deviceIndex = atoi(stringIndex);
			// printf( "INFO      : found device /dev/video%d\n", deviceIndex );
			deviceList.push_back(deviceIndex); // create list of all video devices
		}
	}
	closedir( dp );

	sort(deviceList.begin(), deviceList.end()); // order the /dev/video0 to /dev/videoX

	for( size_t ii = 0; ii < deviceList.size(); ii++ ) {
		sprintf(device, "/dev/video%d", deviceList[ii] );
		// printf( "INFO      : found device %s\n", device );

		cameraHandle = open( device, O_RDWR );
		if( cameraHandle == -1 || cameraHandle == 0 ) {
			printf( "ERROR     : %s when opening capture device %s\n", strerror(errno), device );
			exit( EXIT_FAILURE );
		}

		// get camera capabilities
		struct v4l2_capability caps;
		if( ioctl( cameraHandle, VIDIOC_QUERYCAP, &caps ) == -1 ) {
			printf( "ERROR     : %s when query capabilities of %s \n", strerror(errno), device );
			exit( EXIT_FAILURE );
		}
		close( cameraHandle );

		// show camera capabilities
		// not used values: Linux kernel version, caps.driver, caps.reserved and physical device capabilities
		printf( "INFO      : device %s name %-32s bus %-24s capabilities 0x%08x\n",
				device, caps.card, caps.bus_info, caps.device_caps );

		// the Logitech HD Webcam C525 uses name "HD Webcam C525"
		if( memcmp( caps.card, "HD Webcam C525", 14 ) == 0 ) {
			deviceIndexUsed = deviceList[ii];
		}
	}

	if( deviceIndexUsed == -1 ) {
		printf( "WARNING   : no Logitech HD Webcam C525 has been found, assuming file is used!\n" );
	} else {
		sprintf(device, "/dev/video%d", deviceIndexUsed );
		printf( "INFO      : the Logitech HD Webcam C525 can be accessed through %s\n", device );
		cameraHandle = open( device, O_RDWR );
	}
}

string cameraControl::idToDescription( int id ) {
	if( id == V4L2_CID_BACKLIGHT_COMPENSATION ) { return "backlight compensation"; }
	else if( id == V4L2_CID_BRIGHTNESS ) { return "brightness"; }
	else if( id == V4L2_CID_CONTRAST ) { return  "contrast"; }
	else if( id == V4L2_CID_EXPOSURE_ABSOLUTE ) { return "exposure"; }
	else if( id == V4L2_CID_EXPOSURE_AUTO ) { return  "exposure auto"; }
	else if( id == V4L2_CID_EXPOSURE_AUTO_PRIORITY ) { return  "exposure auto priority"; }
	else if( id == V4L2_CID_FOCUS_ABSOLUTE ) { return  "focus absolute"; }
	else if( id == V4L2_CID_FOCUS_AUTO ) { return  "focus auto"; }
	else if( id == V4L2_CID_GAIN ) { return  "gain"; }
	else if( id == V4L2_CID_LED1_FREQUENCY ) { return  "led1 frequency"; }
	else if( id == V4L2_CID_LED1_MODE ) { return  "led1 mode"; }
	else if( id == V4L2_CID_PAN_ABSOLUTE ) { return  "pan absolute"; }
	else if( id == V4L2_CID_POWER_LINE_FREQUENCY ) { return "power line frequency"; }
	else if( id == V4L2_CID_SATURATION ) { return "saturation"; }
	else if( id == V4L2_CID_SHARPNESS ) { return "sharpness"; }
	else if( id == V4L2_CID_TILT_ABSOLUTE ) { return  "tilt absolute"; }
	else if( id == V4L2_CID_AUTO_WHITE_BALANCE ) { return "white balance auto"; }
	else if( id == V4L2_CID_WHITE_BALANCE_TEMPERATURE ) { return "white balance"; }
	else if( id == V4L2_CID_ZOOM_ABSOLUTE ) { return "zoom absolute"; }
	else { return "unknown ID"; }
}

// get width and height from the camera
void cameraControl::verboseCameraSize( ) {
	struct v4l2_format vfmt;
	memset(&vfmt, 0, sizeof(vfmt));

	vfmt.fmt.pix.priv = 0xfeedcafe; // the v4l2 on the robots does not have V4L2_PIX_FMT_PRIV_MAGIC
	vfmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if( ioctl(cameraHandle, VIDIOC_G_FMT, &vfmt) == -1 ) {
		printf( "ERROR     : %s when trying to get pixel format\n", strerror(errno) );
		exit( EXIT_FAILURE );
	}
	printf( "INFO      : width %u height %u size %u\n", vfmt.fmt.pix.width, vfmt.fmt.pix.height, vfmt.fmt.pix.sizeimage);
}

// get settings value from camera
int cameraControl::get( int id, bool verbose ) {
	control.id = (__u32) id;
	control.value = 0;

	if( ioctl(cameraHandle, VIDIOC_G_CTRL, &control ) == -1 ) {
		printf( "ERROR     : %s when trying to get camera %s value\n", strerror(errno), idToDescription(id).c_str() );
		printf( "ERROR     : check if it is not an old Logitech C525 camera\n" );
		exit( EXIT_FAILURE );
	}
	if( verbose ) {
		printf( "INFO      : %s value is %d\n", idToDescription(id).c_str(), control.value);
	}
	return (int) control.value;
}

// write setting value to camera
void cameraControl::set( int id, int value, bool verbose ) {
	control.id = (__u32) id;
	control.value = (__s32) value;

	if( -1 == ioctl(cameraHandle, VIDIOC_S_CTRL, &control) ) {
		printf( "ERROR     : %s when trying to set camera %s to value %d\n", strerror(errno), idToDescription(id).c_str(), value );
		exit( EXIT_FAILURE );
	}
	if( verbose ) {
		printf( "INFO      : %s set to %d\n", idToDescription(id).c_str(), control.value);
	}
}

void cameraControl::getSettings( ) {
	bool verbose = true;
	get( V4L2_CID_BACKLIGHT_COMPENSATION, verbose );
	get( V4L2_CID_BRIGHTNESS, verbose );
	get( V4L2_CID_CONTRAST, verbose );
	get( V4L2_CID_EXPOSURE_ABSOLUTE, verbose );
	get( V4L2_CID_EXPOSURE_AUTO, verbose );
	get( V4L2_CID_EXPOSURE_AUTO_PRIORITY, verbose );
	get( V4L2_CID_FOCUS_ABSOLUTE, verbose );
	get( V4L2_CID_FOCUS_AUTO, verbose );
	get( V4L2_CID_GAIN, verbose );
	get( V4L2_CID_LED1_FREQUENCY, verbose );
	get( V4L2_CID_LED1_MODE, verbose );
	get( V4L2_CID_PAN_ABSOLUTE, verbose );
	get( V4L2_CID_POWER_LINE_FREQUENCY, verbose );
	get( V4L2_CID_SATURATION, verbose );
	get( V4L2_CID_SHARPNESS, verbose );
	get( V4L2_CID_TILT_ABSOLUTE, verbose );
	get( V4L2_CID_AUTO_WHITE_BALANCE, verbose );
	get( V4L2_CID_WHITE_BALANCE_TEMPERATURE, verbose );
	get( V4L2_CID_ZOOM_ABSOLUTE, verbose );
}

void cameraControl::update( cameraSt camera, bool verbose ) {
	if( deviceIndexUsed >= 0 ) {
		// -1 means no physical camera found, then probably input file used and skip this section

		// check if the camera still works (e.g. because of USB reset or reconnect) by reading one of the values
		get( V4L2_CID_GAIN, false );

		if( configCache.backlightCompensation != camera.backlightCompensation ) {
			set( V4L2_CID_BACKLIGHT_COMPENSATION, camera.backlightCompensation, verbose );
			configCache.backlightCompensation = camera.backlightCompensation;
		}

		if( configCache.brightness != camera.brightness ) {
			set( V4L2_CID_BRIGHTNESS, camera.brightness, verbose );
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

		if( configCache.exposureAutoPriority != camera.exposureAutoPriority ) {
			set( V4L2_CID_EXPOSURE_AUTO_PRIORITY, camera.exposureAutoPriority, verbose );
			configCache.exposureAutoPriority = camera.exposureAutoPriority;
		}

		if( configCache.exposureAbsolute != camera.exposureAbsolute ) {
			if( camera.exposureAuto != 1 ) {
		        // Manual Mode
				if( camera.exposureAbsolute < 3 ) {
					set( V4L2_CID_EXPOSURE_ABSOLUTE, 3, verbose );
				} else {
					set( V4L2_CID_EXPOSURE_ABSOLUTE, camera.exposureAbsolute, verbose );
				}
				gainForceCounter = 0; // after exposure change also set power line frequency and gain
			}
			configCache.exposureAbsolute = camera.exposureAbsolute;
		}

		if( configCache.focusAuto != camera.focusAuto ) {
			set( V4L2_CID_FOCUS_AUTO, camera.focusAuto, verbose );
			if( camera.focusAuto != 1 ) {
		        // only update for Manual Mode
				// in the past to focus was not always set correctly, this is solved by first setting the focus wrong and then apply the correct setting
				set( V4L2_CID_FOCUS_ABSOLUTE, 20, false );
				set( V4L2_CID_FOCUS_ABSOLUTE, camera.focusAbsolute, verbose );
				configCache.focusAbsolute = camera.focusAbsolute;
			}
			configCache.focusAuto = camera.focusAuto;
		} else if( configCache.focusAbsolute != camera.focusAbsolute ) {
			// the focus mode need to be set before the focus absolute
			if( camera.focusAuto != 1 ) {
		        // only update for Manual Mode
				// in the past to focus was not always set correctly, this is solved by first setting the focus wrong and then apply the correct setting
				set( V4L2_CID_FOCUS_ABSOLUTE, 20, false );
				set( V4L2_CID_FOCUS_ABSOLUTE, camera.focusAbsolute, verbose );
			}
			configCache.focusAbsolute = camera.focusAbsolute;
		}

		if( configCache.gain != camera.gain ) {
			gainForceCounter = 0; // combine with setting power line frequency
			configCache.gain = camera.gain;
		}

		if( configCache.led1Frequency != camera.led1Frequency ) {
			// library on robot does not support led set( V4L2_CID_LED1_FREQUENCY, camera.led1Frequency, verbose );
			configCache.led1Frequency = camera.led1Frequency;
		}

		if( configCache.led1Mode != camera.led1Mode ) {
			// library on robot does not support led set( V4L2_CID_LED1_MODE, camera.led1Mode, verbose );
			configCache.led1Mode = camera.led1Mode;
		}

		if( configCache.panAbsolute != camera.panAbsolute ) {
			set( V4L2_CID_PAN_ABSOLUTE, 360*camera.panAbsolute - 36000, verbose );
			configCache.panAbsolute = camera.panAbsolute;
		}

		// this one after the exposure otherwise flickering video
		if( configCache.powerLineFrequency != camera.powerLineFrequency ) {
			// in the past there was a problem with setting the new power line frequency to the camera
			// force by setting first to wrong value
			gainForceCounter = 0; // combine with setting gain
			configCache.powerLineFrequency = camera.powerLineFrequency;
		}

		if( configCache.saturation != camera.saturation ) {
			set( V4L2_CID_SATURATION, camera.saturation, verbose );
			configCache.saturation = camera.saturation;
		}

		if( configCache.sharpness != camera.sharpness ) {
			set( V4L2_CID_SHARPNESS, camera.sharpness, verbose );
			configCache.sharpness = camera.sharpness;
		}

		if( configCache.tiltAbsolute != camera.tiltAbsolute ) {
			set( V4L2_CID_TILT_ABSOLUTE, 360*camera.tiltAbsolute - 36000, verbose );
			configCache.tiltAbsolute = camera.tiltAbsolute;
		}

		if( configCache.whiteBalanceAuto != camera.whiteBalanceAuto ) {
			set( V4L2_CID_AUTO_WHITE_BALANCE, camera.whiteBalanceAuto, verbose );
			if( camera.whiteBalanceAuto != 1 ) {
				if( camera.whiteBalanceTemperature < 2800 ) {
					set( V4L2_CID_WHITE_BALANCE_TEMPERATURE, 2800, verbose );
				} else {
					set( V4L2_CID_WHITE_BALANCE_TEMPERATURE, camera.whiteBalanceTemperature, verbose );
				}
				configCache.whiteBalanceTemperature = camera.whiteBalanceTemperature;
			}
			configCache.whiteBalanceAuto = camera.whiteBalanceAuto;
		} else if( configCache.whiteBalanceTemperature != camera.whiteBalanceTemperature ) {
			if( camera.whiteBalanceAuto != 1 ) {
				if( camera.whiteBalanceTemperature < 2800 ) {
					set( V4L2_CID_WHITE_BALANCE_TEMPERATURE, 2800, verbose );
				} else {
					set( V4L2_CID_WHITE_BALANCE_TEMPERATURE, camera.whiteBalanceTemperature, verbose );
				}
			}
			configCache.whiteBalanceTemperature = camera.whiteBalanceTemperature;
		}

		if( configCache.zoomAbsolute != camera.zoomAbsolute ) {
			if( camera.zoomAbsolute < 1 ) {
				set( V4L2_CID_ZOOM_ABSOLUTE, 1, verbose );
			} else {
				set( V4L2_CID_ZOOM_ABSOLUTE, camera.zoomAbsolute, verbose );
			}
			configCache.zoomAbsolute = camera.zoomAbsolute;
		}

		if( gainForceCounter == 0 ) {
			set( V4L2_CID_POWER_LINE_FREQUENCY, 0, true );
		} else if ( gainForceCounter == 1 ) {
			set( V4L2_CID_POWER_LINE_FREQUENCY, camera.powerLineFrequency, true );
		} else if ( gainForceCounter == 2 ) {
			set( V4L2_CID_POWER_LINE_FREQUENCY, camera.powerLineFrequency, true );
			set( V4L2_CID_GAIN, camera.gain, true );
		}

		if( gainForceCounter < 100 ) {
			gainForceCounter++;
		}

		if( reInitAgainCounter < 20 ) {
			if( reInitAgainCounter == 19 ) {
				printf( "INFO      : reinitializing camera again to prevent brightness difference after camera power cycle\n" );
				clearConfigCache( );
			}
			reInitAgainCounter++;
		}
	}

}
