// Copyright 2021-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef LINEPOINTS_HPP
#define LINEPOINTS_HPP

#include <vector> // std::vector

#include <QImage>
#include <QVector>

#include "config.hpp"
#include "dewarp.hpp"

struct lPointSt { // TODO: provide access to pixelList and then remove lPointSt
   float azimuth; // in radians
   float radius; // in meters
};

class linePoints {
public:
   linePoints(const int robot);
   void update(QImage *camImage[CAMS]);
   std::vector<azimuthRadiusSt> getPixelList() { return linePointsList; };

   QImage *floorImage[CAMS]; // image for green floor pixels
   QImage *lineImage[CAMS]; // image for white line pixels

   // for maximal performance, direct access to the line points array by the search algorithm
   lPointSt array[LINE_POINTS_NUMBER_MAXIMAL];
   int linePointsSize;
   QVector<linePointSt> camLinePoints[CAMS];

   // TODO: find better place for the next variables
   int hueMin[2];
   int hueMax[2];
   int satMin[2];
   int satMax[2];
   int valMin[2];
   int valMax[2];
   int floorMinWidth;
   int floorDec;
   int lineMinWidth;
   int lineDec;

private:
   azimuthRadiusSt cartesianToPolar(const int camera, const int index);
   void createRasterImage();
   void doDewarp( const linePointSt linePoint );
   void filterImage(const QImage *image, const size_t camera);
   bool inRange(const size_t camera, const size_t index);
   QVector<linePointSt> scanLines(const size_t camera, const bool vertical);

   Dewarper *dewarpLookup[CAMS]; // TODO rename to dewarp
   std::vector<azimuthRadiusSt> linePointsList;
   QImage *rasterImage;
   size_t rasterSize;
   int robot;
};

#endif // LINEPOINTS_HPP
