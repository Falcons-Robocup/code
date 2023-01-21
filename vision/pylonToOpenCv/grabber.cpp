// Copyright 2021-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// for pylon camera grab examples see:
// file:///opt/pylon/share/pylon/doc/C++/sample_code.html

// to determine which features are supported by the camera, checkout:
// https://docs.baslerweb.com/features

#include <mutex>
#include <stdio.h>
#include <unistd.h> // usleep

#include <opencv2/opencv.hpp>
#include <pylon/PylonIncludes.h>

#include "grabber.hpp"

using namespace Pylon;
using namespace std;

grabber::grabber() {
	bgrImage = cv::Mat::zeros(CAM_HEIGHT, CAM_WIDTH, CV_8UC3);
	verbose = false;
	frameCounter = 0;
	exportMutex.unlock();

	PylonInitialize(); // initialize pylon runtime interface

	camera = new CInstantCamera(); // overwritten in grabber::attach
}

grabber::~grabber() {
	// reset the camera to assure it will behave correctly for the next test
	printf("INFO    reset camera\n");
	GenApi::INodeMap &nodemap = camera->GetNodeMap();
	CCommandParameter(nodemap, "DeviceReset").Execute();

	PylonTerminate(); // release all pylon resources.
	printf("INFO    grabber all done\n");
}

void grabber::attach(int index) {
	try {
		// get the transport layer factory.
		CTlFactory &tlFactory = CTlFactory::GetInstance();

		// get all attached devices and exit application if no device is found.
		DeviceInfoList_t devices;
		int amount = tlFactory.EnumerateDevices(devices);
		if( amount <= 0 ) {
			throw RUNTIME_EXCEPTION("ERROR   no camera present");
		} else {
			printf("INFO    there are %d cameras available\n", amount);
		}

		if( index < 0 || index > (amount - 1) ) {
			throw RUNTIME_EXCEPTION("ERROR   camera index %d out of range\n", index);
		} else {
			printf("INFO    using camera index %d\n", index);
		}

		camera = new CInstantCamera(tlFactory.CreateDevice(devices[index]));

		cout << "INFO    vendor " << camera->GetDeviceInfo().GetVendorName();
		cout << " model " << camera->GetDeviceInfo().GetModelName();
		serial.assign(camera->GetDeviceInfo().GetSerialNumber()); // serial needs to be available outside this class
		cout << " serial " << serial << endl;
		// cam0 serial 40080490
		// cam1 serial 40149498
		// cam2 serial 40149499
		// cam3 serial 40149529

		if( camera->GetDeviceInfo().IsDeviceVersionAvailable() ) {
			cout << "INFO    version " << camera->GetDeviceInfo().GetDeviceVersion() << endl;
		}

		camera->Open(); // open the camera to get access to the camera registers
		// note: the StartGrabbing also opens the camera, but the configuration shall be performed first

		GenApi::INodeMap &nodemap = camera->GetNodeMap();
		//cout << "INFO    serial " << CStringParameter(nodemap, "DeviceSerialNumber").GetValue() << endl;
		// cam0: firmware version p=du1_imx392c/s=r/v=1.0.2/i=4819.29/h=667793a
		// cam1: firmware version p=du1_imx392c/s=r/v=1.1.0/i=5705.19/h=760308e
		// cam2: firmware version p=du1_imx392c/s=r/v=1.1.0/i=5705.19/h=760308e
		// cam3: firmware version p=du1_imx392c/s=r/v=1.1.0/i=5705.19/h=760308e
		// cout << "INFO    firmware version " << CStringParameter(nodemap, "DeviceFirmwareVersion").GetValue() << endl;
		// cout << "INFO    led status " << CEnumParameter(nodemap, "DeviceIndicatorMode").GetValue() << endl;

		CEnumParameter(nodemap, "DeviceTemperatureSelector").SetValue("FpgaCore");
		float fpgaTemperature = CFloatParameter(nodemap, "DeviceTemperature").GetValue();
		if( fpgaTemperature > 55.0 ) {
			printf("ERROR   FPGA temperature %.1f C is out of range\n", fpgaTemperature);
			exit(EXIT_FAILURE);
		} else {
			printf("INFO    FPGA temperature %.1f C\n", fpgaTemperature);
		}

		// WARNING Sensor temperature not available when streaming !!
		// but except the first basler camera with serial ..., the grabber has to be started and stopped, otherwise warning "Node is not readable"

		// trick to prevent warning:  (this does not occur the first version of the dart camera)
		camera->StartGrabbing(1);
		camera->StopGrabbing();
		CEnumParameter(nodemap, "DeviceTemperatureSelector").SetValue("Sensor");
		float sensorTemperature = CFloatParameter(nodemap, "DeviceTemperature").GetValue();
		if( sensorTemperature > 55.0 ) {
			printf("ERROR   camera sensor temperature %.1f C is out of range\n", sensorTemperature);
			exit(EXIT_FAILURE);
		} else {
			printf("INFO    camera sensor temperature %.1f C\n", sensorTemperature);
		}
	} catch( const GenericException &e ) {
		cerr << "ERROR   an exception occurred" << endl << e.GetDescription() << endl;
		exit(EXIT_FAILURE);
	}
}

void grabber::configure() {
	try {
		// The parameter MaxNumBuffer can be used to control the count of buffers allocated for grabbing.
		// The default value of this parameter is 10.
		// if to low (e.g. 1) then the following error occurs:
		// ERROR   un-successful grab for cam 0, message: Payload data has been discarded.
		// Payload data can be discarded by the camera device if the available bandwidth is insufficient.
		camera->MaxNumBuffer = 20; // default is 10,

		GenApi::INodeMap &nodemap = camera->GetNodeMap();

		CEnumParameter(nodemap, "PixelFormat").SetValue("BayerRG8");
		CIntegerParameter(nodemap, "Width").SetValue(2 * CAM_HEIGHT); // maximal 1920 + 16 (maximal 968 after down scaling)
		CIntegerParameter(nodemap, "Height").SetValue(2 * CAM_WIDTH); // maximal 1200 + 16 (maximal 608 after down scaling)
		CIntegerParameter(nodemap, "OffsetX").SetValue((1920 + 16 - 2 * CAM_HEIGHT) / 2); // center
		CIntegerParameter(nodemap, "OffsetY").SetValue((1200 + 16 - 2 * CAM_WIDTH) / 2); // center

		cout << "INFO    pixel size width " << CIntegerParameter(nodemap, "Width").GetValue();
		cout << " height " << CIntegerParameter(nodemap, "Height").GetValue();
		cout << " offset X " << CIntegerParameter(nodemap, "OffsetX").GetValue();
		cout << " offset Y " << CIntegerParameter(nodemap, "OffsetY").GetValue() << endl;

		CEnumParameter(nodemap, "GainAuto").SetValue("Off");

		// auto adjust as well the white balance, color adjustment and color transformation for the selected light source preset
		// NOTE1: so it is possible to e.g.auto adjust the white balance but do not change the color transformation for a selected source preset
		// NOTE2: the light source presets are calibrated for the IR cut filter in the CS-mount variant of the Basler dart camera
		bool enhancementFeatures = true;
		CEnumParameter(nodemap, "BslLightSourcePresetFeatureSelector").SetValue("WhiteBalance");
		CBooleanParameter(nodemap, "BslLightSourcePresetFeatureEnable").SetValue(enhancementFeatures);
		CEnumParameter(nodemap, "BslLightSourcePresetFeatureSelector").SetValue("ColorAdjustment");
		CBooleanParameter(nodemap, "BslLightSourcePresetFeatureEnable").SetValue(enhancementFeatures);
		CEnumParameter(nodemap, "BslLightSourcePresetFeatureSelector").SetValue("ColorTransformation");
		CBooleanParameter(nodemap, "BslLightSourcePresetFeatureEnable").SetValue(enhancementFeatures);

		CEnumParameter(nodemap, "BslDefectPixelCorrectionMode").SetValue("On"); // On (default), StaticOnly and Off

		float blackLevel = 0.0;
		float exposureTime = 25000.0; // in micro seconds
		float frameRate = 40.0;
		float gain = 1.0;
		float gamma = 1.0;
		string gammaMode = "sRgb"; // nonlinear/Gamma
		string testPattern = "Off";
		// string lightSourcePreset = "Daylight6500K";
		string lightSourcePreset = "Daylight5000K";

		float hue = 0.0;
		float saturation = 1.0;
		float contrast = 0.0;
		float brightness = 0.0;
		float balanceRatioRed = 1.0;
		float balanceRatioGreen = 1.0; // 1.2117;
		float balanceRatioBlue = 1.0; // 1.5676;
		string whiteBalanceMode = "Off"; // Off, Once and Continuous
		// string whiteBalanceMode = "Continuous"; // Off, Once and Continuous

		StringList_t myStringList;

		CFloatParameter(nodemap, "ExposureTime").SetValue(exposureTime);
		cout << "INFO    exposureTime (Min: " << CFloatParameter(nodemap, "ExposureTime").GetMin();
		cout << "; Max: " << CFloatParameter(nodemap, "ExposureTime").GetMax() << ")";
		cout << " set to: " << CFloatParameter(nodemap, "ExposureTime").GetValue() << endl;
		CBooleanParameter(nodemap, "AcquisitionFrameRateEnable").SetValue(false); // use exposure time, instead of framerate, when exposure time is changed

		CFloatParameter(nodemap, "BlackLevel").SetValue(blackLevel);
		cout << "INFO    blacklevel (Min: " << CFloatParameter(nodemap, "BlackLevel").GetMin();
		cout << "; Max: " << CFloatParameter(nodemap, "BlackLevel").GetMax() << ")";
		cout << " set to: " << CFloatParameter(nodemap, "BlackLevel").GetValue() << endl;

		CFloatParameter(nodemap, "Gain").SetValue(gain);
		cout << "INFO    gain (Min: " << CFloatParameter(nodemap, "Gain").GetMin();
		cout << "; Max: " << CFloatParameter(nodemap, "Gain").GetMax() << ")";
		cout << " set to: " << CFloatParameter(nodemap, "Gain").GetValue() << endl;

		CFloatParameter(nodemap, "Gamma").SetValue(gamma);
		cout << "INFO    gamma (Min: " << CFloatParameter(nodemap, "Gamma").GetMin();
		cout << "; Max: " << CFloatParameter(nodemap, "Gamma").GetMax() << ")";
		cout << " set to: " << CFloatParameter(nodemap, "Gamma").GetValue() << endl;

		CEnumParameter(nodemap, "BslColorSpace").SetValue(gammaMode.c_str());
		CEnumParameter(nodemap, "BslColorSpace").GetSettableValues(myStringList);
		cout << "INFO    gammaMode options:";
		for( size_t ii = 0; ii < myStringList.size(); ii++ ) {
			cout << " " << myStringList[ii];
		}
		cout << " set to: " << CEnumParameter(nodemap, "BslColorSpace").GetValue() << endl;

		CFloatParameter(nodemap, "AcquisitionFrameRate").SetValue(frameRate);
		cout << "INFO    frameRate (Min: " << CFloatParameter(nodemap, "AcquisitionFrameRate").GetMin();
		cout << "; Max: " << CFloatParameter(nodemap, "AcquisitionFrameRate").GetMax() << ")";
		cout << " set to: " << CFloatParameter(nodemap, "AcquisitionFrameRate").GetValue() << endl;
		CBooleanParameter(nodemap, "AcquisitionFrameRateEnable").SetValue(true); // use framerate, instead of exposure time, when framerate is changed

		CEnumParameter(nodemap, "TestPattern").SetValue(testPattern.c_str());
		CEnumParameter(nodemap, "TestPattern").GetSettableValues(myStringList);
		cout << "INFO    testPattern options:";
		for( size_t ii = 0; ii < myStringList.size(); ii++ ) {
			cout << " " << myStringList[ii];
		}
		cout << " set to: " << CEnumParameter(nodemap, "TestPattern").GetValue() << endl;

		CEnumParameter(nodemap, "BslLightSourcePreset").GetSettableValues(myStringList);
		cout << "INFO    lightSourcePreset options:";
		for( size_t ii = 0; ii < myStringList.size(); ii++ ) {
			cout << " " << myStringList[ii];
		}
		CEnumParameter(nodemap, "BslLightSourcePreset").SetValue(lightSourcePreset.c_str());
		cout << "; set to: " << CEnumParameter(nodemap, "BslLightSourcePreset").GetValue() << endl;

		CFloatParameter(nodemap, "BslHue").SetValue(hue);
		cout << "INFO    hue (Min: " << CFloatParameter(nodemap, "BslHue").GetMin();
		cout << "; Max: " << CFloatParameter(nodemap, "BslHue").GetMax() << ")";
		cout << " set to: " << CFloatParameter(nodemap, "BslHue").GetValue() << endl;

		CFloatParameter(nodemap, "BslSaturation").SetValue(saturation);
		cout << "INFO    saturation (Min: " << CFloatParameter(nodemap, "BslSaturation").GetMin();
		cout << "; Max: " << CFloatParameter(nodemap, "BslSaturation").GetMax() << ")";
		cout << " set to " << CFloatParameter(nodemap, "BslSaturation").GetValue() << endl;

		CFloatParameter(nodemap, "BslContrast").SetValue(contrast);
		cout << "INFO    contrast (Min: " << CFloatParameter(nodemap, "BslContrast").GetMin();
		cout << "; Max: " << CFloatParameter(nodemap, "BslContrast").GetMax() << ")";
		cout << " set to: " << CFloatParameter(nodemap, "BslContrast").GetValue() << endl;

		CFloatParameter(nodemap, "BslBrightness").SetValue(brightness);
		cout << "INFO    brightness (Min: " << CFloatParameter(nodemap, "BslBrightness").GetMin();
		cout << "; Max: " << CFloatParameter(nodemap, "BslBrightness").GetMax() << ")";
		cout << " set to: " << CFloatParameter(nodemap, "BslBrightness").GetValue() << endl;

		CEnumParameter(nodemap, "BalanceRatioSelector").SetValue("Red");
		CFloatParameter(nodemap, "BalanceRatio").SetValue(balanceRatioRed);
		cout << "INFO    balance ratio red (Min: " << CFloatParameter(nodemap, "BalanceRatio").GetMin();
		cout << "; Max: " << CFloatParameter(nodemap, "BalanceRatio").GetMax() << ")";
		cout << " set to: " << CFloatParameter(nodemap, "BalanceRatio").GetValue() << endl;

		CEnumParameter(nodemap, "BalanceRatioSelector").SetValue("Green");
		CFloatParameter(nodemap, "BalanceRatio").SetValue(balanceRatioGreen);
		cout << "INFO    balance ratio green (Min: " << CFloatParameter(nodemap, "BalanceRatio").GetMin();
		cout << "; Max: " << CFloatParameter(nodemap, "BalanceRatio").GetMax() << ")";
		cout << " set to: " << CFloatParameter(nodemap, "BalanceRatio").GetValue() << endl;

		CEnumParameter(nodemap, "BalanceRatioSelector").SetValue("Blue");
		CFloatParameter(nodemap, "BalanceRatio").SetValue(balanceRatioBlue);
		cout << "INFO    balance ratio blue (Min: " << CFloatParameter(nodemap, "BalanceRatio").GetMin();
		cout << "; Max: " << CFloatParameter(nodemap, "BalanceRatio").GetMax() << ")";
		cout << " set to: " << CFloatParameter(nodemap, "BalanceRatio").GetValue() << endl;

		CEnumParameter(nodemap, "BalanceWhiteAuto").SetValue(whiteBalanceMode.c_str());
		CEnumParameter(nodemap, "BalanceWhiteAuto").GetSettableValues(myStringList);
		cout << "INFO    white balance mode options:";
		for( size_t ii = 0; ii < myStringList.size(); ii++ ) {
			cout << " " << myStringList[ii];
		}
		cout << " set to: " << CEnumParameter(nodemap, "BalanceWhiteAuto").GetValue() << endl;
	} catch( const GenericException &e ) {
		cerr << "ERROR   an exception occurred" << endl << e.GetDescription() << endl;
		exit(EXIT_FAILURE);
	}
}

void grabber::start() {
	// continue grabbing frames
	// TODO: find out what capture strategy to use
	// file:///opt/pylon/share/pylon/doc/C++/group___pylon___instant_camera_api_generic.html#gga56a922b1bd37848234153b9e12f7fecbacb541053c64948f5d55e7955588deb33
	// file:///opt/pylon/share/pylon/doc/C++/group___pylon___instant_camera_api_generic.html#ggaf582b0a1fa8604d4e18857bd525291e6a1006c968199ded056843b9c42854b206
	// camera->StartGrabbing(GrabStrategy_LatestImages);
	camera->StartGrabbing(GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByUser);
}

void grabber::update() {
	while( camera->IsGrabbing() ) {

		// wait for an image and then retrieve it
		// a timeout of 5000 ms is used
		camera->RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);

		if( ptrGrabResult->GrabSucceeded() ) {
			// get the image data
			const uint8_t *pImageBuffer = (uint8_t*)ptrGrabResult->GetBuffer();

			if( verbose ) {
				printf("INFO    framecounter %8zu\n", frameCounter++);
				printf("INFO    grab successful %1d %1d %1d %1d\n", ptrGrabResult->GetWidth(),
						ptrGrabResult->GetHeight(), ptrGrabResult->GetOffsetX(), ptrGrabResult->GetOffsetY());
				printf("INFO    image size %zu\n", ptrGrabResult->GetImageSize());
				EPixelType myType = ptrGrabResult->GetPixelType();
				printf("INFO    bit depth %d bits per pixel %d\n", BitDepth(myType), BitPerPixel(myType));
				printf("INFO    value for first pixel %d %d %d\n", pImageBuffer[0], pImageBuffer[1], pImageBuffer[2]);
			}

			downscale_1_1_2(pImageBuffer);
		} else {
			cout << "Error: " << hex << ptrGrabResult->GetErrorCode() << dec << " "
					<< ptrGrabResult->GetErrorDescription() << endl;
		}
	}
}

// bayer to cv::Mat conversion and rotate 90 degrees (camera is mounted for portrait mode)
// properties:
//  - best sharpness
//  - downscale factor 2x2
//  - one red and blue pixel and 2 green pixels
//  - this requires about 8% less CPU time then INTERPOLATION_5_5_6
void grabber::downscale_1_1_2(const uint8_t *buff) {
	// combine 4 bayer pixels to one rgb pixel
	exportMutex.lock();
	for( int ii = 0; ii < 2 * CAM_WIDTH; ii += 2 ) { // ii < 1216
		for( int kk = 0; kk < 2 * CAM_HEIGHT; kk += 2 ) { // kk < 1936
			int index = ii * 2 * CAM_HEIGHT + 2 * CAM_HEIGHT - 2 - kk;
			int red = buff[index + 0];
			int green = buff[index + 1];
			int blue = buff[index + 2 * CAM_HEIGHT + 1]; // blue pixel is on next line
			green += buff[index + 2 * CAM_HEIGHT + 0]; // second green pixel on next line
			bgrImage.at<cv::Vec3b>(kk / 2, ii / 2) = cv::Vec3b(blue, green / 2, red); // opencv requires bgr instead of rgb
		}
	}
	exportMutex.unlock();
}

string grabber::getSerial() {
	return serial;
}

cv::Mat grabber::getImage() {
	exportMutex.lock();
	bgrImage.copyTo(exportImage);
	exportMutex.unlock();
	return exportImage;
}
