 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2015-2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// Uses the green floor to calibrate the floor
// The main targets are to set the white balance, brightness and saturation of the camera
// The brightness of the camera can be controlled by 3 parameters:
// - brightness
// - exposure
// - gain
//
// Note: the first generation Logitech c525 cameras do not support brightness and gain
//
// The calculation can be done every frame (20 fps)
//
// Changing the camera settings is performed by the v4l2-ctl external program
// This can probably not be performed on 20 fps
// So for now let's go for about an update every second
//
// Note: sometimes the camera does not update to the latest setting with usage of v4l2-ctl

// Functionality:
// Filter out the green floor with a "wide" color filter
// Measure the HSV average off all green pixels
// Use the Hue value to change the camera white balance
// Use the Sat value to change the camera saturation
// Use the Value to change the intensity parameters
// The 3 intensity parameters are:
// gain: boost light in low-light conditions, causes graininess to the image
// exposure: lens open time, bringing more light, but also can create a blurry movement effect
// brightness: makes the whole picture brighter or darker (where contrast make the dark darker and bright brighter), it looks like more white is added
// so for our purpose brightness is not usable because it makes recognizing of lines and obstacles worse.

// Some measurement values
//
// evening light, begin evening not that much light, white manual 6236, auto 5962
// evening light, pretty bright, white manual 6100, auto 6024
// TL light and evening light, white manual 5570, black manual 6000, auto 5218
// TL + HG light and evening light, white manual 5550, black manual 6100, auto 5280

// It might be needed to add a  moving average filter and or hysteresis
// before changing the camera settings.

#include "dynamicCalibration.hpp"
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <cerrno>
#include <iomanip>
#include <sys/stat.h>

using namespace cv;
using namespace std;

// not defined in V4L2 but from logitech.xml
#define V4L2_CID_LED1_MODE 0x0A046D05
#define V4L2_CID_LED1_FREQUENCY 0x0A046D06

dynamicCalibration::dynamicCalibration( configurator *conf = NULL, preprocessor *prep = NULL)
{
	this->conf = conf;
	this->prep = prep;
	floorPixelsPrev = 0;
	gainModeFrames = 0;
	whiteBalanceTemperatureCalibration = false;
	whiteBalanceTemperatureUp = true;
	whiteBalanceTemperatureBest = 0;
	lineSatMaxDefault = 0;
	fd = 0;
	powerLineSetFreq = 0;
	snprintf(currentDate, sizeof currentDate, "undefined");
	snprintf(currentTime, sizeof currentTime, "undefined");
	snprintf(fileName, sizeof fileName, "undefined");
	snprintf(fileNameBest, sizeof fileName, "undefined");
	pixelsBest = 0;

	struct timeval tv;
	time_t nowtime;
	struct tm *nowtm;

	gettimeofday(&tv, NULL);
	nowtime = tv.tv_sec;
	nowtm = localtime(&nowtime);
	strftime(currentDate, sizeof currentDate, "%Y-%m-%d", nowtm);
	// use current date as directory name
	struct stat sbuf;
	if( lstat( currentDate, &sbuf ) == -1 ) {
		cout << "create directory " << currentDate << " to store dynamic calibration statistics images" << endl;
		if( mkdir(currentDate, 0775 ) ) {
			cerr << "cannot create directory " << currentDate << endl;
		}
	}
	// use current time as file name
	strftime(currentTime, sizeof currentTime, "%H:%M:%S", nowtm);
	snprintf(fileNameCSV, sizeof fileName, "%s/%s.csv", currentDate, currentTime);

	statsFile.open(fileNameCSV);
	statsFile << "date,time,frame,uptime,fps,gain,whiteBal,linePixels,lineHue,lineSat,lineVal,floorPixels,floorHue,floorSat,floorVal,ballPixels,ballHue,ballSat,ballVal" << endl;
}

dynamicCalibration::~dynamicCalibration()
{
	statsFile.close();
}

// get settings value from camera
int dynamicCalibration::get(string setName, int id)
{
	struct v4l2_control control;
	control.id = (__u32) id;
	control.value = 0;

	if (fd == 0 ) {
		cerr << "multiCam: dynamicCalibration Error file descriptor to camera not set" << endl;
		return 0;
	}

	if (-1 == ioctl (fd, VIDIOC_G_CTRL, &control)) {
		cerr << std::endl;
		cerr << "multiCam: dynamicCalibration Error for getting VIDIOC_G_CTRL through ioctl for id " << setName << " (0x" << hex << id << dec << ")" << endl;
		cerr << "Error: " << strerror(errno) << endl;
		exit (EXIT_FAILURE);
	}
	return (int) control.value;
}

// write setting value to camera
void dynamicCalibration::set(string setName, int id, int value)
{
	struct v4l2_control control;
	control.id = (__u32) id;
	control.value = (__s32) value;

	if (fd == 0 ) {
		cerr << "multiCam: dynamicCalibration Error file descriptor to camera not set" << endl;
		return;
	}

	if (-1 == ioctl (fd, VIDIOC_S_CTRL, &control)) {
		cerr << endl;
		cerr << "multiCam: dynamicCalibration Error for setting VIDIOC_S_CTRL through ioctl for id " << setName << " (0x" << hex << id << dec << ") with value " << value << endl;
		cerr << "Error: " << strerror(errno) << endl;
		cerr << "The camera control only works for newer Logitech c525 camera's the early c525 lack a number of essential controls" << endl;
		exit (EXIT_FAILURE);
	}
}

// update and check one of the used camera controls
void dynamicCalibration::nameSet(string setName, int id, int prevVal, int newVal, bool verbose )
{
	if( id == V4L2_CID_FOCUS_ABSOLUTE ) {
		// the step size for focus is 5 instead of 1
		newVal = 5 * (newVal / 5 );
		if( prevVal != -1 ) {
			prevVal = 5 * (prevVal / 5 );
		}
	}

	if( id == V4L2_CID_WHITE_BALANCE_TEMPERATURE ) {
		if( get( "white balance auto", V4L2_CID_AUTO_WHITE_BALANCE) == 1 ) {
			// cerr << "multiCam: dynamicCalibration Error not able to set white balance because auto white balance is enabled" << endl;
			return;
		}
	}


	if( prevVal != newVal ) {
		if( verbose ) {
			if( prevVal == -1 ) {
				cout << "Set " << setName << " to " << setw(3) << newVal << endl;
			} else {
				cout << "Update " << setName << " from " << setw(3) << prevVal << " to " << setw(3) << newVal << endl;
			}
		}
		// write value to camera
		set( setName, id, newVal );
		// check if new value has been changed in the camera
		int readBack = get( setName, id );
		if( newVal != readBack ) {
			cout << "multiCam: dynamicCalibration error: " << setName << " is " << setw(4) << readBack << " but should be " << setw(4) << newVal << endl;
		}
	}
}

// run only once when the camera is active
void dynamicCalibration::initialize(int fd)
{
	this->fd = fd; // for use with other get and set functions
	struct v4l2_capability caps; // APOX with " = {0}; " results in warnings on ubuntu 12.04 / ros

	// show type of camera
	if (-1 == ioctl(fd, VIDIOC_QUERYCAP, &caps))
	{
		cerr << endl;
		cerr << "multiCam: dynamicCalibration Error when querying capabilities" << endl;
		cerr << "Error: " << strerror(errno) << endl;
		exit (EXIT_FAILURE);
	}

	int versionMajor = (int)(caps.version>>16)&&0xff;
	int versionMinor = (int)(caps.version>>24)&&0xff;
	cout << "driver " << caps.driver << ", card " << caps.card << ", bus " << caps.bus_info;
	cout << ", version " << versionMajor << "." << versionMinor;
	cout << ", reserved " <<  caps.reserved;
// APOX: not available on ubuntu 12.04	cout << ", device caps 0x" << hex << caps.device_caps;
	cout << ", capabilities 0x" << hex << caps.capabilities << dec << endl;

	// early type logitech c525:
	// driver uvcvideo, card HD Webcam C525, bus usb-0000:00:14.0-4, version 1.0, reserved 0x7fff40d9ee2c, device caps 0x4000001, capabilities 0x84000001
	// windows (with logitech software) reports pid 0x0826, bcd 0010, vid 0x046d, asic version 162, firmware version 6.50.1065, firmware crc 0xc6ba (50874), eeprom version 2.93 and senors type 2.0 for this camera
	// type used on robot:
	// driver uvcvideo, card HD Webcam C525, bus usb-0000:00:14.0-4, version 1.0, reserved 0x7fff59ccd8cc, device caps 0x4000001, capabilities 0x84000001
	// windows (with logitech software) reports pid 0x0826, bcd 0010, vid 0x046d, asic version 166, firmware version 6.50.1072, firmware crc 0xafa5 (44965), eeprom version 2.93 and senors type 2.0 for this camera
	// the reserved number changes each run (maybe a pointer)
	// so this can not be used to distinguish
	// but dmesg shows a serial number e.g. robot6 : usb 3-13: SerialNumber: 47C02660


	// set the camera defaults
	// this is specific for this type of logitech c525 usb camera
	// another usb camera probably has a different set of controls !
	bool verbose = true;
	cameraSt camera = conf->getCamera();
	nameSet( "backlight compensation", V4L2_CID_BACKLIGHT_COMPENSATION, -1, camera.backlightCompensation, verbose );
	nameSet( "brightness", V4L2_CID_BRIGHTNESS, -1, camera.brightness, verbose );
	nameSet( "contrast", V4L2_CID_CONTRAST, -1, camera.contrast, verbose );
	nameSet( "exposure auto", V4L2_CID_EXPOSURE_AUTO, -1, camera.exposureAuto, verbose );
	nameSet( "exposure priority", V4L2_CID_EXPOSURE_AUTO_PRIORITY, -1, camera.exposureAutoPriority, verbose );
	nameSet( "exposure", V4L2_CID_EXPOSURE_ABSOLUTE, -1, camera.exposureAbsolute, verbose ); // after auto and auto priority
	nameSet( "focus auto", V4L2_CID_FOCUS_AUTO, -1, camera.focusAuto, verbose );
	nameSet( "focus", V4L2_CID_FOCUS_ABSOLUTE, -1, 20, 0 ); // after focus auto, this wrong setting is required to activate the next setting
	nameSet( "focus", V4L2_CID_FOCUS_ABSOLUTE, -1, camera.focusAbsolute, verbose ); // after focus auto
	nameSet( "gain", V4L2_CID_GAIN, -1, camera.gain, verbose );
// APOX todo: does not work on robot	nameSet( "led1Frequency", V4L2_CID_LED1_FREQUENCY, -1, camera.led1Frequency, verbose ); // not available in v4l2 library, requires logitech library
// APOX todo: does not work on robot	nameSet( "led1Mode", V4L2_CID_LED1_MODE, -1, camera.led1Mode, verbose ); // not available in v4l2 library, requires logitech library
	nameSet( "power line frequency", V4L2_CID_POWER_LINE_FREQUENCY, -1, 0, verbose ); // to get it correct working, toggle first to disable
	nameSet( "power line frequency", V4L2_CID_POWER_LINE_FREQUENCY, -1, 2, verbose ); // to get it correct working, toggle first to disable
	nameSet( "power line frequency", V4L2_CID_POWER_LINE_FREQUENCY, -1, 0, verbose ); // to get it correct working, toggle first to disable
	nameSet( "power line frequency", V4L2_CID_POWER_LINE_FREQUENCY, -1, 2, verbose ); // to get it correct working, toggle first to disable
	nameSet( "power line frequency", V4L2_CID_POWER_LINE_FREQUENCY, -1, camera.powerLineFrequency, verbose ); // this one after the exposure otherwise flickering video
	nameSet( "power line frequency", V4L2_CID_POWER_LINE_FREQUENCY, -1, camera.powerLineFrequency, verbose ); // this one after the exposure otherwise flickering video
	nameSet( "power line frequency", V4L2_CID_POWER_LINE_FREQUENCY, -1, camera.powerLineFrequency, verbose ); // this one after the exposure otherwise flickering video
	nameSet( "saturation", V4L2_CID_SATURATION, -1, camera.saturation, verbose );
	nameSet( "sharpness", V4L2_CID_SHARPNESS, -1, camera.sharpness, verbose );
	nameSet( "white balance auto", V4L2_CID_AUTO_WHITE_BALANCE, -1, camera.whiteBalanceAuto, verbose );
	nameSet( "white balance", V4L2_CID_WHITE_BALANCE_TEMPERATURE, -1, camera.whiteBalanceTemperature, verbose ); // after white balance auto
	nameSet( "zoom", V4L2_CID_ZOOM_ABSOLUTE, -1, camera.zoomAbsolute, verbose );

	cameraPrev = camera; // cameraPrev is used to check if one or more of the camera settings has been changed
}

// run this function for every frame (20 fps)
void dynamicCalibration::update()
{
	// capture the current line, floor and ball color, of which some are used by other members of this class (e.g. logging)
	lineColor = cv::mean( prep->getHsv(), ( prep->getLine() != 0 ) );
	floorColor = cv::mean( prep->getHsv(), ( prep->getFloor() != 0 ) );
	ballColor = cv::mean( prep->getHsv(), ( prep->getBall() != 0 ) );

	cameraSt camera = conf->getCamera();
	bool verbose = true;
	nameSet( "brightness", V4L2_CID_BRIGHTNESS, cameraPrev.brightness, camera.brightness, verbose );
	nameSet( "contrast", V4L2_CID_CONTRAST, cameraPrev.contrast, camera.contrast, verbose );
	nameSet( "exposure", V4L2_CID_EXPOSURE_ABSOLUTE, cameraPrev.exposureAbsolute, camera.exposureAbsolute, verbose );
	nameSet( "focus", V4L2_CID_FOCUS_ABSOLUTE, cameraPrev.focusAbsolute, camera.focusAbsolute, verbose );
	nameSet( "saturation", V4L2_CID_SATURATION, cameraPrev.saturation, camera.saturation, verbose );
	nameSet( "sharpness", V4L2_CID_SHARPNESS, cameraPrev.sharpness, camera.sharpness, verbose );
	nameSet( "white balance", V4L2_CID_WHITE_BALANCE_TEMPERATURE, cameraPrev.whiteBalanceTemperature, camera.whiteBalanceTemperature, verbose );
	nameSet( "zoom", V4L2_CID_ZOOM_ABSOLUTE, cameraPrev.zoomAbsolute, camera.zoomAbsolute, verbose );

#ifdef NONO
	// disable auto gain
	if( ( prep->getFloorPixels() > 200 ) && ! whiteBalanceTemperatureCalibration ) {
		// Update gain depending on the value of the (green) floor.
		// Only update if enough floor pixels and the white balance calibration is not running
		// because the white balance will interfere with the foorColor.val[2] (value).
		// A (green) floor value between 130 and 150 results in a picture which is not to dark and not to bright.
		// APOX todo: checkout if around 140 is the best value
		if( floorColor.val[2] < 130 ) { // value
			if( camera.gain < 255 ) {
				camera.gain++;
			}
		}
		if( floorColor.val[2] > 150 ) {
			if( camera.gain > 0 ) {
				camera.gain--;
			}
		}
	}
#endif

	nameSet( "gain", V4L2_CID_GAIN, cameraPrev.gain, camera.gain, verbose );

#ifdef NONO
	// with a fluorescent lamp and halogen lamp this does not work because it is not possible
	// to get the hue back with changing the white balance.
	int hue = conf->getFloor().hue.center;
	if( floorColor.val[0] < hue ) { // hue
		if( camera.whiteBalanceTemperature > ( 2800 + 10 ) ) {
			camera.whiteBalanceTemperature = camera.whiteBalanceTemperature - 10;
		}
	}
	if( floorColor.val[0] > hue ) {
		if( camera.whiteBalanceTemperature < ( 6500 - 10 ) ) {
			camera.whiteBalanceTemperature = camera.whiteBalanceTemperature + 10;
		}
	}
	nameSet( "white balance", V4L2_CID_WHITE_BALANCE_TEMPERATURE, cameraPrev.whiteBalanceTemperature, camera.whiteBalanceTemperature, 1 );
#endif

#ifdef NONO
	// poor man's implementation for maximum search, but it works for now
	if( prep->getFloorPixels() < floorPixelsPrev ) {
		whiteBalanceUp = ! whiteBalanceUp;
	}
	floorPixelsPrev = prep->getFloorPixels();

	if( whiteBalanceUp ) {
		if( camera.whiteBalanceTemperature < ( 6500 - 10 ) ) {
			camera.whiteBalanceTemperature = camera.whiteBalanceTemperature + 10;
		}
	} else {
		if( camera.whiteBalanceTemperature > ( 2800 + 10 ) ) {
			camera.whiteBalanceTemperature = camera.whiteBalanceTemperature - 10;
		}
	}
	nameSet( "white balance", V4L2_CID_WHITE_BALANCE_TEMPERATURE, cameraPrev.whiteBalanceTemperature, camera.whiteBalanceTemperature, 0 );
#endif
    // when changing the exposure the gain is also influenced and the video start flickering
	// so when change the exposure toggle the line frequency again and set back the gain
	// however this "resets" the exposure value again.
	// it appears that only 3 exposure windows can be used:
	// below 200
	// around 270
	// above 360
	if( cameraPrev.exposureAbsolute != camera.exposureAbsolute ) {
		nameSet( "power line frequency", V4L2_CID_POWER_LINE_FREQUENCY, -1, 0, 0 ); // to get it correct working, toggle first to disable
		nameSet( "power line frequency", V4L2_CID_POWER_LINE_FREQUENCY, -1, camera.powerLineFrequency, 0 );
		nameSet( "gain", V4L2_CID_GAIN, -1, camera.gain, 0 );
	}


	if( powerLineSetFreq == 5 ) {
		nameSet( "power line frequency", V4L2_CID_POWER_LINE_FREQUENCY, -1, 0, true ); // this one after the exposure otherwise flickering video
	} else if( powerLineSetFreq == 10 ) {
		nameSet( "power line frequency", V4L2_CID_POWER_LINE_FREQUENCY, -1, camera.powerLineFrequency, true ); // this one after the exposure otherwise flickering video
	} else if( powerLineSetFreq == 15 ) {
		nameSet( "power line frequency", V4L2_CID_POWER_LINE_FREQUENCY, -1, camera.powerLineFrequency, true ); // this one after the exposure otherwise flickering video
		nameSet( "gain", V4L2_CID_GAIN, -1, camera.gain, verbose );
	}

	if( powerLineSetFreq < 1000 ) {
		powerLineSetFreq++;
	}

	cameraPrev = camera; // update the previous values for the next call
	conf->setCamera( camera ); // write back dynamic updated values to configuration e.g. gain
}

int dynamicCalibration::getWhiteBalanceTemperature()
{
	return get( "white balance", V4L2_CID_WHITE_BALANCE_TEMPERATURE);
}

int dynamicCalibration::getGain(){
	return get( "gain", V4L2_CID_GAIN);
}

void dynamicCalibration::captureStats(uint64 frame){

	// csv list:
	// date,time,frame,uptime,framesPerSecond,gain,whiteBalance,linePixels,lineHue,lineSat,lineVal,floorPixels,floorHue,floorSat,floorVal
	statsFile << currentDate << ",";
	statsFile << currentTime << ",";

	statsFile << frame << ",";

	timeSt timeData = prep->getTime();
	double uptime = (timeData.fieldTime[0] - timeData.startTime )/getTickFrequency();
	statsFile << fixed << setprecision(1) << uptime << ",";

	int fieldTimeSize = sizeof(timeData.fieldTime) / sizeof(timeData.fieldTime[0]);
	double fps = fieldTimeSize * 1.0 /((timeData.fieldTime[0] - timeData.fieldTime[fieldTimeSize-1])/getTickFrequency());
	statsFile << fixed << setprecision(1) << fps << ",";

	statsFile << getGain() << ",";
	statsFile << getWhiteBalanceTemperature() << ",";

	statsFile << prep->getLinePixels() << ",";
	statsFile << lineColor.val[0] << ","; // hue
	statsFile << lineColor.val[1] << ","; // sat
	statsFile << lineColor.val[2] << ","; // val

	statsFile << prep->getFloorPixels() << ",";
	statsFile << floorColor.val[0] << ","; // hue
	statsFile << floorColor.val[1] << ","; // sat
	statsFile << floorColor.val[2] << ","; // val

	statsFile << prep->getBallPixels() << ",";
	statsFile << ballColor.val[0] << ","; // hue
	statsFile << ballColor.val[1] << ","; // sat
	statsFile << ballColor.val[2]; // val

	statsFile << endl;
}

void dynamicCalibration::sweep(int frameCounter){

	timeSt timeData = prep->getTime();
	uint64 frame;
	if( frameCounter >= 0 ) {
		frame = (uint64) frameCounter;
	} else {
		frame = timeData.fieldCount;
	}
	if( whiteBalanceTemperatureCalibration && ( frame % 20 ) == 0 ) {
		// wait a few frames to let the new white balance value settle

		// line sat max on 1
		struct timeval tv;
		time_t nowtime;
		struct tm *nowtm;

		gettimeofday(&tv, NULL);
		nowtime = tv.tv_sec;
		nowtm = localtime(&nowtime);
		strftime(currentDate, sizeof currentDate, "%Y-%m-%d", nowtm);
		// use current date as directory name
		struct stat sbuf;
		if( lstat( currentDate, &sbuf ) == -1 ) {
			cout << "create directory " << currentDate << " to store dynamic calibration statistics images" << endl;
			if( mkdir(currentDate, 0775 ) ) {
				cerr << "cannot create directory " << currentDate << endl;
			}
		}

	    // use current time as file name
		strftime(currentTime, sizeof currentTime, "%H:%M:%S", nowtm);
		snprintf(fileName, sizeof fileName, "%s/%s.png", currentDate, currentTime);

		// store information in csv file
		captureStats( frame );
		cameraSt camera = conf->getCamera(); // get whiteBalanceTemperature value

		// int pixels = prep->getLinePixels();
		int pixels = prep->getFloorPixels();
		if( pixels > pixelsBest ) {
			pixelsBest = pixels;
			strcpy( fileNameBest,fileName );
			prep->getBgr().copyTo( imageBest );
			whiteBalanceTemperatureBest = camera.whiteBalanceTemperature; // use as new default used during gain adjustment phase
		}

	    // new white balance value
	    bool storeBest = false;
	    if( whiteBalanceTemperatureUp ) {
	    	if( camera.whiteBalanceTemperature >= 6400 ) {
	    		// hit the upper end (when going up)
	    		whiteBalanceTemperatureUp = false;
	    		storeBest = true;
	    	}
	    } else {
	    	if( camera.whiteBalanceTemperature <= 4000 ) {
	    		// the the lower end (when going down)
	    		whiteBalanceTemperatureUp = true;
	    		storeBest = true;
	    		 // done with white balance calibration, switch to gain mode
				cout << "switch to gain mode" << endl;
	    		whiteBalanceTemperatureCalibration = false;
				conf->setLineSatMaxDebug( lineSatMaxDefault ); // increase the line saturation window again for localization
	    	}
	    }

	    if( storeBest && pixelsBest > 0 ) {
	    	// when pixels best still 0, then no picture / interesting data available
	    	imwrite(fileNameBest, imageBest);
	    	pixelsBest = 0;
	    }

	    // prepare for next cycle
	    if( whiteBalanceTemperatureCalibration ) {
		    if( whiteBalanceTemperatureUp ) {
				camera.whiteBalanceTemperature = camera.whiteBalanceTemperature + 50;
			} else {
				camera.whiteBalanceTemperature = camera.whiteBalanceTemperature - 50;
			}
	    } else {
			camera.whiteBalanceTemperature = whiteBalanceTemperatureBest; // use the best white balance for gain control
	    }

	    conf->setCamera( camera ); // write back dynamic updated values to configuration e.g. gain

		// line sat max on slider value
		// enable auto gain
		// sleep 10 minutes
	}

	if( ! whiteBalanceTemperatureCalibration ) {
		// so we are in gain adjustment mode
		// wait for 200 frames to switch back to white balance calibration mode
		if( gainModeFrames > 200 ) {
			gainModeFrames = 0;
			cout << "switch to white balance mode" << endl;
			// done with gain mode, switch to white balance mode
			whiteBalanceTemperatureCalibration = true;
			// set white balance camera to lowest value before starting the sweep
			cameraSt camera = conf->getCamera(); // get whiteBalanceTemperature value
			camera.whiteBalanceTemperature = 4000; // start on lower end
			conf->setCamera( camera ); // write back dynamic updated values to configuration e.g. gain
			lineSatMaxDefault = conf->getLine().sat.max;
			conf->setLineSatMaxDebug( 2 ); // reduce saturation window for white lines (with correct white balance there should be no color)
		} else {
			gainModeFrames++;
		}
	}
}
