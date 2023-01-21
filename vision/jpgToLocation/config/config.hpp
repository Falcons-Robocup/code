// Copyright 2021-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <QPoint>

#define CAMS 4

#define CAM_WIDTH 608
#define CAM_HEIGHT 800

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

#endif // CONFIG_HPP

