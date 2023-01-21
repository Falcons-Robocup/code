// Copyright 2021-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <QPoint>
#include <QString>
#include <pylon/PylonIncludes.h>

#define USE_JPG

// the bayer converter (software) reduces the pixes by half for width and height
#define CAM_WIDTH 608 // max 608 (1216)
// TODO: for now on 800 to be compatible with dewarp table created for raspi cameras
// increase CAM_HEIGHT back to e.g. 864 to be able to detect balls near the robot
#define CAM_HEIGHT 800 // max 968 (1936)
// #define CAM_HEIGHT 864 // max 968 (1936)

#define CAMS 4 // 4 camera's

#define FLOOR_WIDTH 1000 // 18+2 meter / 1000 = 2 cm per pixel
#define FLOOR_HEIGHT 700 // 12+2 meter / 700 = 2 cm per pixel

#define LINE_POINTS_NUMBER_MINIMAL 40 // minimal amount of linepoints that have been used
#define LINE_POINTS_NUMBER_MAXIMAL 100 // maximal amount of linepoints used as input search

struct linePointSt {
   QPoint begin;
   QPoint end;
};

struct azimuthRadiusSt {
   double azimuth;
   double radius;
   size_t xCamera;
   size_t yCamera;
   uint16_t xFloor;
   uint16_t yFloor;
   bool valid;
   bool operator<(const azimuthRadiusSt& val) const {
      // sorting this struct is performed on radius (distance)
      return radius > val.radius;
   }
};

typedef struct {
   double score; // TODO: remove
   int amountOnFloor;
   int amountOffFloor;
   bool positionFound;
   double confidence;
   double confidenceLevel; // TODO remove
} scoreStruct;

typedef struct {
   double x;
   double y;
   double rz;
} positionStDbl;

class config {
public:
   config(Pylon::CInstantCameraArray *cameras);
   void init();
   void update();
   void reset();
   double getFpgaTemp(size_t cam) { return fpgaTemperatures[cam]; }

   Pylon::CInstantCameraArray *cameras; // generic camera interface array

   double blackLevel, blackLevelPrev; // 0 to 1023, only upto 100 makes sense
   double exposureTime, exposureTimePrev; // 19 us to 10.000.000 us (10 seconds)
   double gamma, gammaPrev; // 0 to 3.99998
   QString gammaMode, gammaModePrev; // Off and sRgb
   double detFps; // 0.1 to 1.000.000 (with BayerRG8, usb3 and 1920x1200 maximal 130 fps) // TODO: move to detector.hpp
   double frameRate, frameRatePrev;
   double gain, gainPrev; // 0 to 48.0 - 48.0 * FLT_EPSILON ~ 0 to 47.99999
   QString testPattern, testPatternPrev;
   QString lightSourcePreset, lightSourcePresetPrev;
   double hue, huePrev;
   double saturation, saturationPrev;
   double contrast, contrastPrev;
   double brightness, brightnessPrev;
   double balanceRatioRed, balanceRatioRedPrev;
   double balanceRatioRedActual;
   double balanceRatioGreen, balanceRatioGreenPrev;
   double balanceRatioGreenActual;
   double balanceRatioBlue, balanceRatioBluePrev;
   double balanceRatioBlueActual;
   QString whiteBalanceMode, whiteBalanceModePrev;

   double fpgaTemperatures[CAMS];
};

#endif // CONFIG_HPP
