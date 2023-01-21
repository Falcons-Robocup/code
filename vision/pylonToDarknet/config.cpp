// Copyright 2021-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <cfloat> // for float defines
#include <unistd.h>

#include "config.hpp"

using namespace GenApi; // TODO: disable namespace
using namespace Pylon;
// using namespace Basler_UniversalCameraParams;
using namespace std;

config::config(Pylon::CInstantCameraArray *cameras) {
   this->cameras = cameras;

   // set to invalid for use previous value compare
   blackLevel = FLT_MIN;
   blackLevelPrev = 0.0;
   exposureTime = FLT_MIN;
   exposureTimePrev = 0.0;
   frameRate = FLT_MIN;
   frameRatePrev = 0.0;
   detFps = 1.0;
   gain = FLT_MIN;
   gainPrev = 0.0;
   gamma = FLT_MIN;
   gammaPrev = 0.0;
   gammaMode = "unset";
   gammaModePrev = "zero";
   testPattern = "unset";
   testPatternPrev = "zero";
   lightSourcePreset = "unset";
   lightSourcePresetPrev = "zero";

   hue = FLT_MIN;
   huePrev = 0.0;
   saturation = FLT_MIN;
   saturationPrev = 0.0;
   contrast = FLT_MIN;
   contrastPrev = 0.0;
   brightness = FLT_MIN;
   brightnessPrev = 0.0;
   balanceRatioRed = FLT_MIN;
   balanceRatioRedPrev = 0.0;
   balanceRatioRedActual = FLT_MIN;
   balanceRatioGreen = FLT_MIN;
   balanceRatioGreenPrev = 0.0;
   balanceRatioGreenActual = FLT_MIN;
   balanceRatioBlue = FLT_MIN;
   balanceRatioBluePrev = 0.0;
   balanceRatioBlueActual = FLT_MIN;
   whiteBalanceMode = "unset";
   whiteBalanceModePrev = "zero";

   for( size_t cam = 0; cam < CAMS; cam++) {
      fpgaTemperatures[cam]= FLT_MIN;
   }
}

void config::init() {
   reset();

   StringList_t myStringList;

#ifndef USE_JPG
   cameras->Open(); // the camera shall be open before performing write actions (note: the grabber also opens the camera, but configuration is first)

   for( size_t kk = 0; kk < cameras->GetSize(); kk++ ) {
      try {
         // using camera device 0 of 4 daA1920-160uc serial 40080490
         // using camera device 1 of 4 daA1920-160uc serial 40149498
         // using camera device 2 of 4 daA1920-160uc serial 40149499
         // using camera device 3 of 4 daA1920-160uc serial 40149529
         GenApi::INodeMap &nodemap = cameras->operator[](kk).GetNodeMap();
         //        cout << "camera information ";
         //        cout << CStringParameter( nodemap, "DeviceVendorName" ).GetValue() << " ";
         //        cout << CStringParameter( nodemap, "DeviceModelName" ).GetValue() << " ";
         //        cout << CStringParameter( nodemap, "DeviceSerialNumber" ).GetValue() << endl;

         // cout << CIntegerParameter( nodemap, "DeviceLinkThroughputLimit" ).GetValue() << endl;
         // CCommandParameter(nodemap, "TimestampLatch").Execute();
         // cout << CIntegerParameter( nodemap, "TimestampLatchValue" ).GetValue() << endl;

         // one time setters
         // CEnumParameter( nodemap, "ExposureAuto" ).SetValue("Off"); // TODO, options: Off Once Continuous
         CEnumParameter(nodemap, "ExposureAuto").GetSettableValues(myStringList);
         cout << "ExposureAuto options:";
         for (size_t ii = 0; ii < myStringList.size(); ii++) {
            cout << " " << myStringList[ii];
         }
         cout << endl;

         // CEnumParameter( nodemap, "ExposureMode" ).SetValue("Timed"); // TODO, options: Timed TriggerWidth TriggerControlled
         CEnumParameter(nodemap, "ExposureMode").GetSettableValues(myStringList);
         cout << "ExposureMode options:";
         for (size_t ii = 0; ii < myStringList.size(); ii++) {
            cout << " " << myStringList[ii];
         }
         cout << endl;

         // configure the camera (these are likely already the defaults)
         CEnumParameter( nodemap, "PixelFormat" ).SetValue("BayerRG8");
         // cout << "pixel format " << CEnumParameter( nodemap, "PixelFormat" ).GetValue() << endl;

         CIntegerParameter( nodemap, "Width" ).SetValue(2*CAM_HEIGHT); // maximal 1920 + 16 (maximal 968 after downscaling)
         CIntegerParameter( nodemap, "Height" ).SetValue(2*CAM_WIDTH); // maximal 1200 + 16 (maximal 608 after downscaling)
         CIntegerParameter( nodemap, "OffsetX" ).SetValue((1920+16-2*CAM_HEIGHT)/2); // center
         CIntegerParameter( nodemap, "OffsetY" ).SetValue((1200+16-2*CAM_WIDTH)/2); // center

         cout << "pixel size width " << CIntegerParameter( nodemap, "Width").GetValue();
         cout << " height " << CIntegerParameter( nodemap, "Height").GetValue();
         cout << " offset X " << CIntegerParameter( nodemap, "OffsetX").GetValue();
         cout << " offset Y " << CIntegerParameter( nodemap, "OffsetY").GetValue() << endl;

         CEnumParameter( nodemap, "GainAuto" ).SetValue("Off");

         // auto adjust as well the whitebalance, color adjustment and color transformation for the selected light source preset
         // NOTE1: so it is possible to e.g.auto adjust the white balance but do not change the color transformation for a selected source preset
         // NOTE2: the light source presets are calibrated for the IR cut filter in the CS-mount variant of the Basler dart camera
         bool enhancementFeatures = true;
         CEnumParameter( nodemap, "BslLightSourcePresetFeatureSelector" ).SetValue("WhiteBalance");
         CBooleanParameter( nodemap, "BslLightSourcePresetFeatureEnable" ).SetValue(enhancementFeatures);
         CEnumParameter( nodemap, "BslLightSourcePresetFeatureSelector" ).SetValue("ColorAdjustment");
         CBooleanParameter( nodemap, "BslLightSourcePresetFeatureEnable" ).SetValue(enhancementFeatures);
         CEnumParameter( nodemap, "BslLightSourcePresetFeatureSelector" ).SetValue("ColorTransformation");
         CBooleanParameter( nodemap, "BslLightSourcePresetFeatureEnable" ).SetValue(enhancementFeatures);

         CEnumParameter( nodemap, "BslDefectPixelCorrectionMode" ).SetValue("On"); // On (default), StaticOnly and Off

      } catch (const GenericException& e) {
         cerr << "An exception occurred." << endl << e.GetDescription() << endl;
      }
   }
#endif
}

void config::reset() {
   blackLevel = 0.0;
   exposureTime = 25000.0; // in micro seconds
   frameRate = 40.0;
   gain = 1.0;
   gamma = 1.0;
   gammaMode = "sRgb"; // nonlinear/Gamma
   testPattern = "Off";
   // lightSourcePreset = "Daylight6500K";
   lightSourcePreset = "Daylight5000K";

   hue = 0.0;
   saturation = 1.0;
   contrast = 0.0;
   brightness = 0.0;
   balanceRatioRed = 1.0;
   balanceRatioGreen = 1.0; // 1.2117;
   balanceRatioBlue = 1.0; // 1.5676;
   whiteBalanceMode = "Off"; // Off, Once and Continuous
   // whiteBalanceMode = "Continuous"; // Off, Once and Continuous
}

void config::update() {
#ifndef USE_JPG
   StringList_t myStringList;
   size_t availableCams = cameras->GetSize();
   if( availableCams > CAMS ) {
      cerr << "ERROR   there are " << availableCams << "  avaialble cameras, but expected only " << CAMS << endl;
      exit(1);
   }
   for( size_t kk = 0; kk < availableCams; kk++ ) {
      try {
         GenApi::INodeMap &nodemap = cameras->operator[](kk).GetNodeMap();

         CEnumParameter(nodemap, "DeviceTemperatureSelector").SetValue("FpgaCore");

         fpgaTemperatures[kk] = CFloatParameter(nodemap, "DeviceTemperature").GetValue();

         // NOTE Sensor temperature not availalbe when streaming !!
         // CEnumParameter(nodemap, "DeviceTemperatureSelector").SetValue("Sensor");
         // sensorTemperature = CFloatParameter(nodemap, "DeviceTemperature").GetValue();

         if( ( exposureTime != exposureTimePrev ) ) {
            CFloatParameter( nodemap, "ExposureTime" ).SetValue(exposureTime);
            cout << "exposureTime (Min: " << CFloatParameter(nodemap, "ExposureTime").GetMin();
            cout << "; Max: " << CFloatParameter(nodemap, "ExposureTime").GetMax() << ")";
            cout << " set to: " << CFloatParameter(nodemap, "ExposureTime").GetValue() << endl;
            CBooleanParameter( nodemap, "AcquisitionFrameRateEnable" ).SetValue(false); // use exposure time, instead of framerate, when exposure time is changed
         }
         if( ( blackLevel != blackLevelPrev ) ) {
            CFloatParameter( nodemap, "BlackLevel" ).SetValue(blackLevel);
            cout << "blacklevel (Min: " << CFloatParameter(nodemap, "BlackLevel").GetMin();
            cout << "; Max: " << CFloatParameter(nodemap, "BlackLevel").GetMax() << ")";
            cout << " set to: " << CFloatParameter(nodemap, "BlackLevel").GetValue() << endl;
         }
         if( ( gain != gainPrev ) ) {
            CFloatParameter( nodemap, "Gain" ).SetValue(gain);
            cout << "gain (Min: " << CFloatParameter(nodemap, "Gain").GetMin();
            cout << "; Max: " << CFloatParameter(nodemap, "Gain").GetMax() << ")";
            cout << " set to: " << CFloatParameter(nodemap, "Gain").GetValue() << endl;
         }

         if( ( gamma != gammaPrev ) ) {
            CFloatParameter( nodemap, "Gamma" ).SetValue(gamma);
            cout << "gamma (Min: " << CFloatParameter(nodemap, "Gamma").GetMin();
            cout << "; Max: " << CFloatParameter(nodemap, "Gamma").GetMax() << ")";
            cout << " set to: " << CFloatParameter(nodemap, "Gamma").GetValue() << endl;
         }

         if( ( gammaMode != gammaModePrev ) ) {
            CEnumParameter( nodemap, "BslColorSpace" ).SetValue(gammaMode.toStdString().c_str());
            CEnumParameter(nodemap, "BslColorSpace").GetSettableValues(myStringList);
            cout << "gammaMode options:";
            for (size_t ii = 0; ii < myStringList.size(); ii++) {
               cout << " " << myStringList[ii];
            }
            cout << " set to: " << CEnumParameter(nodemap, "BslColorSpace").GetValue() << endl;
         }

         if( ( frameRate != frameRatePrev ) ) {
            CFloatParameter( nodemap, "AcquisitionFrameRate" ).SetValue(frameRate);
            cout << "frameRate (Min: " << CFloatParameter(nodemap, "AcquisitionFrameRate").GetMin();
            cout << "; Max: " << CFloatParameter(nodemap, "AcquisitionFrameRate").GetMax() << ")";
            cout << " set to: " << CFloatParameter(nodemap, "AcquisitionFrameRate").GetValue() << endl;
            CBooleanParameter( nodemap, "AcquisitionFrameRateEnable" ).SetValue(true); // use framerate, instead of exposure time, when framerate is changed
         }

         if( ( testPattern != testPatternPrev ) ) {
            CEnumParameter( nodemap, "TestPattern" ).SetValue(testPattern.toStdString().c_str());
            CEnumParameter(nodemap, "TestPattern").GetSettableValues(myStringList);
            cout << "testPattern options:";
            for (size_t ii = 0; ii < myStringList.size(); ii++) {
               cout << " " << myStringList[ii];
            }
            cout << " set to: " << CEnumParameter(nodemap, "TestPattern").GetValue() << endl;
         }

         if( ( lightSourcePreset != lightSourcePresetPrev ) ) {
            CEnumParameter(nodemap, "BslLightSourcePreset").GetSettableValues(myStringList);
            cout << "lightSourcePreset options:";
            for (size_t ii = 0; ii < myStringList.size(); ii++) {
               cout << " " << myStringList[ii];
            }
            CEnumParameter( nodemap, "BslLightSourcePreset" ).SetValue(lightSourcePreset.toStdString().c_str());
            cout << "; set to: " << CEnumParameter(nodemap, "BslLightSourcePreset").GetValue() << endl;
         }

         if( ( hue != huePrev ) ) {
            CFloatParameter( nodemap, "BslHue" ).SetValue(hue);
            cout << "hue (Min: " << CFloatParameter(nodemap, "BslHue").GetMin();
            cout << "; Max: " << CFloatParameter(nodemap, "BslHue").GetMax() << ")";
            cout << " set to: " << CFloatParameter(nodemap, "BslHue").GetValue() << endl;
         }


         if( ( saturation != saturationPrev ) ) {
            CFloatParameter( nodemap, "BslSaturation" ).SetValue(saturation);
            cout << "saturation (Min: " << CFloatParameter(nodemap, "BslSaturation").GetMin();
            cout << "; Max: " << CFloatParameter(nodemap, "BslSaturation").GetMax() << ")";
            cout << " set to " << CFloatParameter(nodemap, "BslSaturation").GetValue() << endl;
         }

         if( ( contrast != contrastPrev ) ) {
            CFloatParameter( nodemap, "BslContrast" ).SetValue(contrast);
            cout << "contrast (Min: " << CFloatParameter(nodemap, "BslContrast").GetMin();
            cout << "; Max: " << CFloatParameter(nodemap, "BslContrast").GetMax() << ")";
            cout << " set to: " << CFloatParameter(nodemap, "BslContrast").GetValue() << endl;
         }

         if( ( brightness != brightnessPrev ) ) {
            CFloatParameter( nodemap, "BslBrightness" ).SetValue(brightness);
            cout << "brightness (Min: " << CFloatParameter(nodemap, "BslBrightness").GetMin();
            cout << "; Max: " << CFloatParameter(nodemap, "BslBrightness").GetMax() << ")";
            cout << " set to: " << CFloatParameter(nodemap, "BslBrightness").GetValue() << endl;
         }


         if( ( balanceRatioRed != balanceRatioRedPrev ) ) {
            CEnumParameter( nodemap, "BalanceRatioSelector" ).SetValue("Red");
            CFloatParameter( nodemap, "BalanceRatio" ).SetValue(balanceRatioRed);
            cout << "balance ratio red (Min: " << CFloatParameter(nodemap, "BalanceRatio").GetMin();
            cout << "; Max: " << CFloatParameter(nodemap, "BalanceRatio").GetMax() << ")";
            cout << " set to: " << CFloatParameter(nodemap, "BalanceRatio").GetValue() << endl;
         }
         CEnumParameter( nodemap, "BalanceRatioSelector" ).SetValue("Red");
         balanceRatioRedActual = CFloatParameter(nodemap, "BalanceRatio").GetValue();

         if( ( balanceRatioGreen != balanceRatioGreenPrev ) ) {

            CEnumParameter( nodemap, "BalanceRatioSelector" ).SetValue("Green");
            CFloatParameter( nodemap, "BalanceRatio" ).SetValue(balanceRatioGreen);
            cout << "balance ratio green (Min: " << CFloatParameter(nodemap, "BalanceRatio").GetMin();
            cout << "; Max: " << CFloatParameter(nodemap, "BalanceRatio").GetMax() << ")";
            cout << " set to: " << CFloatParameter(nodemap, "BalanceRatio").GetValue() << endl;
         }
         CEnumParameter( nodemap, "BalanceRatioSelector" ).SetValue("Green");
         balanceRatioGreenActual = CFloatParameter(nodemap, "BalanceRatio").GetValue();

         if( ( balanceRatioBlue != balanceRatioBluePrev ) ) {

            CEnumParameter( nodemap, "BalanceRatioSelector" ).SetValue("Blue");
            CFloatParameter( nodemap, "BalanceRatio" ).SetValue(balanceRatioBlue);
            cout << "balance ratio blue (Min: " << CFloatParameter(nodemap, "BalanceRatio").GetMin();
            cout << "; Max: " << CFloatParameter(nodemap, "BalanceRatio").GetMax() << ")";
            cout << " set to: " << CFloatParameter(nodemap, "BalanceRatio").GetValue() << endl;
         }
         CEnumParameter( nodemap, "BalanceRatioSelector" ).SetValue("Blue");
         balanceRatioBlueActual = CFloatParameter(nodemap, "BalanceRatio").GetValue();

         if( ( whiteBalanceMode != whiteBalanceModePrev ) ) {
            CEnumParameter( nodemap, "BalanceWhiteAuto" ).SetValue(whiteBalanceMode.toStdString().c_str());
            CEnumParameter(nodemap, "BalanceWhiteAuto").GetSettableValues(myStringList);
            cout << "white balance mode options:";
            for (size_t ii = 0; ii < myStringList.size(); ii++) {
               cout << " " << myStringList[ii];
            }
            cout << " set to: " << CEnumParameter(nodemap, "BalanceWhiteAuto").GetValue() << endl;
         }
      } catch (const GenericException& e) {
         // Error handling.
         cerr << "An exception occurred." << endl
              << e.GetDescription() << endl;
      }

      if( kk == availableCams - 1 ) { // all camera's shall be updated with new value, keep invalid until the last camera is configured
         exposureTimePrev = exposureTime;
         blackLevelPrev = blackLevel;
         gainPrev = gain;
         gammaPrev = gamma;
         gammaModePrev = gammaMode;
         frameRatePrev = frameRate;
         testPatternPrev = testPattern;
         lightSourcePresetPrev = lightSourcePreset;
         huePrev = hue;
         saturationPrev = saturation;
         contrastPrev = contrast;
         brightnessPrev = brightness;
         balanceRatioRedPrev = balanceRatioRed;
         balanceRatioGreenPrev = balanceRatioGreen;
         balanceRatioBluePrev = balanceRatioBlue;
         whiteBalanceModePrev = whiteBalanceMode;
      }
   }
#endif
}
