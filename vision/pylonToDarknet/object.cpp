// Copyright 2021-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <QDebug>
#include <QtMath> // for qRadiansToDegrees

#include <cfloat> // for FLT_EPSILON

#include "object.hpp"
#include "config.hpp"

object::object() {
   azimuth = 0.0;
   camera = 0;
   classId = 0;
   confidence= 0.0;
   elevation = 0.0;
   height = 0.0;
   isDewarped = false;
   name = "unset";
   nameShort  ="unset";
   radius = 0.0;
   robot = 0;
   valid = false;
   width = 0.;
   xCenter = 0.0;
   yCenter = 0.0;
   color = Qt::red;
}

void object::update(int camera, Dewarper *dewarpLookup, int robot) { // TODO rename to dewarp
   this->camera = camera;
   this->dewarpLookup = dewarpLookup;
   this->robot = robot;
   rangeCheck();
   doDewarp();
}

void object::rangeCheck() {
   // range check
   if ( confidence < 0.0 || confidence >= 1.0 ) {
      qDebug() << "ERROR   confidence" << confidence << "out of range";
      valid = false;
   }

   if ( height >= 1.0 && height < 1.16127 ) {
      // still acceptable, because YOLO can send height values up to 1.16127 instead of < 1
      height = 1.0 - 1.0 * FLT_EPSILON;
   } else if ( height < 0.0 || height >= 1.0 ) {
      qDebug() << "ERROR   height" << height << "out of range";
      valid = false;
   }

   if ( width >= 1.0 && width < 1.2 ) {
      // still acceptable, because YOLO can send width values up to 1.078 instead of < 1
      width = 1.0 - 1.0 * FLT_EPSILON;
   } else if ( width < 0.0 || width >= 1.0 ) {
      qDebug() << "ERROR   width" << width << "out of range";
      valid = false;
   }

   if( xCenter < 0.0 || xCenter >= 1.0 ) {
      qDebug() << "ERROR   xCenter" << xCenter << "out of range";
      valid = false;
   }

   if( yCenter < 0.0 || yCenter >= 1.0 ) {
      qDebug() << "ERROR   yCenter" << yCenter << "out of range";
      valid = false;
   }

   // check classId range and set name and pen color
   switch ( classId ) {
   case 0 :
      // qDebug() << "INFO    set ball color to yellow";
      name = "ball";
      nameShort = "ball ";
      color = Qt::yellow;
      break;
   case 1 :
      name = "obstacle";
      nameShort = "obsta";
      color = 0xde4c8a; // near erika violet
      // color = 0xcf618c; // ? erika violet
      // color = 0xf486bd; // erika violet
      break;
   case 2 :
      name = "human";
      nameShort = "human";
      color = 0xff9900; // orange
      break;
   case 3 :
      name = "border";
      nameShort = "bordr";
      color = 0x81d4fa; // light blue
      break;
   case 4 :
      name = "goal post";
      nameShort = "goalp";
      color = Qt::white;
      break;
   default:
      name = "unkwown";
      nameShort = "unkwn";
      color = Qt::red;
      qDebug() << "ERROR   class id" << classId << "out of range";
      valid = false;
   }
}

void object::doDewarp() {
   if ( ! valid ) { return; }

   uint16_t xCenterPixel = (int) (CAM_WIDTH * xCenter); // range 0 to 607
   if (xCenterPixel > (CAM_WIDTH-1)) {
      qDebug() << "ERROR   xCenter" << xCenterPixel << "out of range" << CAM_WIDTH-1;
   }

   uint16_t yCenterPixel = (int) (CAM_HEIGHT * yCenter); // range 0 to 799
   if (yCenterPixel > (CAM_HEIGHT-1)) {
      qDebug() << "ERROR   yCenter" << yCenterPixel << "out of range" << CAM_HEIGHT-1;

   }

   // y = 0 is at top of image, but transformFront expects y = 0 at bottom of image
   yCenterPixel = (CAM_HEIGHT - 1) - yCenterPixel;
   isDewarped = dewarpLookup->transformFront(yCenterPixel, xCenterPixel, azimuth, elevation);
   // azimuth definition: 0 degrees is nort and 90 degrees is east

   /*
        if ( classId == 1 ) {
            qDebug().noquote() << QString().asprintf("x %3d y %3d azimuth %3.0f deg elevation %3.0f deg",
                                                    xCenterPixel, yCenterPixel,
                                                    qRadiansToDegrees(azimuth), qRadiansToDegrees(elevation));
        }
        */

   // simple way to estimate the radius, based on the closest pixel on the y axis
   // calculate distance using dewarp floor lookup tables
   radius = FLT_MAX;				// initialize, FLT_MAX indicates radius unknown
   float yClosestRel = yCenter + height / 2.0;
   // bring back in range: yCenter combined with height/2 might be out of range
   if (yClosestRel >= 1.0) {
      yClosestRel = 1.0 - 1.0 * FLT_EPSILON; // range does not include 1.0 itself
   }
   // y = 0 is at top of image, but transformFront expects y = 0 at bottom of image
   int yClosestBy = (CAM_HEIGHT - 1) - (int) (CAM_HEIGHT * yClosestRel); // range 0 to 799

   // try to find closest by for the most left, center and right x position
   float xLeftRel = xCenter - width / 2.0;
   // bring back in range: xCenter combined with width/2 might be out of range
   if (xLeftRel < 0.0) {
      xLeftRel = 0.0;
   }
   int xLeft = (int) (CAM_WIDTH * xLeftRel); // range 0 to 607
   if (xLeft > (CAM_WIDTH-1)) {
      qDebug() << "ERROR   xLeft" << xLeft << "out of range xleftRel" << xLeftRel << "xCenter" << xCenter << "width" << width;
   }

   float xRightRel = xCenter + width / 2.0;
   // might get out of range
   if (xRightRel >= 1.0) {
      xRightRel = 1.0 - 1.0 * FLT_EPSILON; // range does not include 1.0 itself
   }
   int xRight = (int) (CAM_WIDTH * xRightRel); // range 0 to 607
   if (xRight > (CAM_WIDTH-1)) {
      qDebug() << "ERROR   xRight" << xRight << "out of range xRightRel" << xRightRel << "xCenter" << xCenter << "width" << width;
   }

   int16_t xField, yField; // results

   // find the lowes radius for xLeft, xCenter and xRight
   if (dewarpLookup->transformFloor(yClosestBy, xLeft, yField, xField)) {
      // xField and yField result in mm
      float xLeftRadius = 0.001 * sqrt(xField * xField + yField * yField);
      if (xLeftRadius < radius) {
         radius = xLeftRadius; // radius in meters
      }
   }

   if (dewarpLookup->transformFloor(yClosestBy, xCenterPixel, yField, xField)) {
      // xField and yField result in mm
      float xCenterRadius = 0.001 * sqrt(xField * xField + yField * yField);
      if (xCenterRadius < radius) {
         radius = xCenterRadius; // radius in meters
      }
   }

   if (dewarpLookup->transformFloor(yClosestBy, xRight, yField, xField)) {
      // xField and yField result in mm
      float xRightRadius = 0.001 * sqrt(xField * xField + yField * yField);
      if (xRightRadius < radius) {
         radius = xRightRadius; // radius in meters
      }
   }

   /*
        if (classId == 2) {
            QString text;
            text.append(QString().asprintf("INFO    xCenter %3d yClosest %3d xField %4.1f m yField %4.1f m",
                                          xCenterPixel, yClosestBy, xField / 1000.0, yField / 1000.0));

            if (radius == FLT_MAX) {
                text.append(" radius   MAX m");
            } else {
                text.append(QString().asprintf(" radius %5.2f m", radius));
            }
            qDebug().noquote() << text;
        }
        */

   if( robot > 8 ) {
      if( camera == 1 || camera == 3 ) {
         azimuth = 0.5 * M_PI + azimuth; // pylon camera's are clockwise mounted instead counter clockwise and might be calibrated incorrect
      }
   }

   if (! isDewarped ) {
      qDebug() << "ERROR   unexpected failure in dewarp->transformFront for xCenter" << xCenterPixel << "yCenter" << yCenterPixel;
      azimuth = 0.0;
      elevation = 0.0;
      radius = 0.0;
   } // success
}

QString object::getPrintString(bool shortName) {
   QString text;
   if( shortName ) {
      text.append(nameShort);
   } else {
      text.append(name);
   }

   text.append( QString().asprintf(" x %4.2f y %4.2f w %4.2f h %4.2f conf %3.0f%% azi %4.0f deg ele %4.0f deg",
                                   xCenter, yCenter, width, height, 100.0 * confidence,
                                   qRadiansToDegrees(azimuth), qRadiansToDegrees(elevation)));

   if ( radius == FLT_MAX ) {
      text.append(" rad   MAX m");
   } else {
      text.append( QString().asprintf(" rad %5.2f m", radius));
   }

   return text;
}

void object::print(bool shortname = true) {
   qDebug().noquote() << getPrintString(shortname);
}


