// Copyright 2014-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <QDebug>

#include "fieldLut.hpp"

fieldLut::fieldLut(linePoints *lPoints, robotFloor *rFloor) {
   this->lPoints = lPoints;
   this->rFloor = rFloor;

   xScale = rFloor->getXScale(); // convert between relative and meters
   yScale = rFloor->getYScale();
   // qDebug() << "xScale" << xScale << "yScale" << yScale;

   xLeft = rFloor->getFieldSize().xLeft - 0.015; // 0.015 is about 0.30 meter
   xRight = rFloor->getFieldSize().xRight + 0.015;
   yTop = rFloor->getFieldSize().yTop - 0.020; // 0.020 is about 0.28 meter
   yBottom = rFloor->getFieldSize().yBottom + 0.020;
}

// confidence range 0.0 to 1.0, higher is better
double fieldLut::calc(const double *x) const { // TODO: change to float
   int fieldPointsAmount = 0; // dirty way to export information
   int fieldPointsOutsideField = 0;
   int confidenceSum = 0;
   float confidence = 0.0;
   // calculates new points using x (x[0]), y (x[1]) and rz (x[2])
   for( int ii = 0; ii < lPoints->linePointsSize ; ii++ ) {
      // azimuth definition: 0 degrees is north and 90 degrees is east
      float azimuth = lPoints->array[ii].azimuth + x[2]; // azimuth range from -2 Pi to 2 Pi, Note: 2 Pi because addition rZ

      float pxMeters = lPoints->array[ii].radius * std::sin(azimuth); // radius in meters, if on field then range about -20 meter to 20 meter
      float pyMeters = - lPoints->array[ii].radius * std::cos(azimuth); // y is top to bottom
      float pxRelative = pxMeters * xScale; // if on field then range -1.0 to 1.0
      float pyRelative = pyMeters * yScale; // if on field then range -1.0 to 1.0

      float px = x[0] + pxRelative;
      float py = x[1] + pyRelative;
      // only calculate the cost of pixels that are on the floor (field + (part of) border)
      if( px >= xLeft && px <= xRight && py >= yTop && py <= yBottom ) {
         confidenceSum += rFloor->costTable[(int)(px * FLOOR_WIDTH)][(int)(py * FLOOR_HEIGHT)]; // add value of each found pixel to the confidence
         fieldPointsAmount++;
      } else {
         // qDebug("px %3d py %3d x left %3d x rigth %3d y top %3d y bottom %3d", py, px, xLeft, xRight, yTop, yBottom);
         fieldPointsOutsideField++;
      }
   }

   // only perform the position search if there are enough linepoints
   if( fieldPointsAmount >= LINE_POINTS_NUMBER_MINIMAL ) {
      confidence = 1.0 * confidenceSum / (fieldPointsAmount * 256.0); // calculate confidence and normalize between 0.0 and 1.0

      // make the confidence slightly worse for each pixels that is outside the field (to prevent a good confidence while most pixels are outside the field)
      // a good lock has a confidence higher then 95%
      // a bad lock has a confidence lower then 82%
      // 30 points outside the field should result result in a worse confidence
      // so set 30 points outside the field to a confidence decrease of 15%, that means each point subtracks 0.5% from the confidence
      confidence -= fieldPointsOutsideField*0.005;

      if( confidence < 0.0 ) { confidence = 0.0; } // assure the subtrack cannot result in negative confidence
   } else {
      // not enough line points to determine the position
      confidence = 0.0;
   }

   // qDebug("INFO    linepoints %3d inside %3d outside %3d sum %5d confidence %6.3f",
   //       lPoints->linePointsSize, fieldPointsAmount, fieldPointsOutsideField, confidenceSum, confidence);

   return 1.0 - confidence; // the solver requires the minimal value instead of close to 1.0
}

// confidence range 0.0 to 1.0, higher is better
scoreStruct fieldLut::calcSimple(const float *x) {
   int fieldPointsAmount = 0; // dirty way to export information
   int fieldPointsOutsideField = 0;
   int confidenceSum = 0;
   float confidence = 0.0;
   // calculates new points using x (x[0]), y (x[1]) and rz (x[2])
   for( int ii = 0; ii < lPoints->linePointsSize ; ii++ ) {
      // azimuth definition: 0 degrees is north and 90 degrees is east
      float azimuth = lPoints->array[ii].azimuth + x[2]; // azimuth range from -2 Pi to 2 Pi, Note: 2 Pi because addition rZ

      float pxMeters = lPoints->array[ii].radius * std::sin(azimuth); // radius in meters, if on field then range about -20 meter to 20 meter
      float pyMeters = - lPoints->array[ii].radius * std::cos(azimuth); // y is top to bottom
      float pxRelative = pxMeters * xScale; // if on field then range -1.0 to 1.0
      float pyRelative = pyMeters * yScale; // if on field then range -1.0 to 1.0

      float px = x[0] + pxRelative;
      float py = x[1] + pyRelative;
      // only calculate the cost of pixels that are on the floor (field + (part of) border)
      if( px >= xLeft && px <= xRight && py >= yTop && py <= yBottom ) {
         confidenceSum += rFloor->costTable[(int)(px * FLOOR_WIDTH)][(int)(py * FLOOR_HEIGHT)]; // add value of each found pixel to the confidence
         fieldPointsAmount++;
      } else {
         // qDebug("px %3d py %3d x left %3d x rigth %3d y top %3d y bottom %3d", py, px, xLeft, xRight, yTop, yBottom);
         fieldPointsOutsideField++;
      }
   }

   // only perform position search if there are enough linepoints
   if( fieldPointsAmount >= LINE_POINTS_NUMBER_MINIMAL ) {
      confidence = 1.0 * confidenceSum / (fieldPointsAmount * 256.0); // calculate confidence and normalize between 0.0 and 1.0

      // make the confidence slightly worse for each pixels that is outside the field (to prevent a good confidence while most pixels are outside the field)
      // a good lock has a confidence higher then 95%
      // a bad lock has a confidence lower then 82%
      // 30 points outside the field should result result in a worse confidence
      // so set 30 points outside the field to a confidence decrease of 15%, that means each point subtracks 0.5% from the confidence
      confidence -= fieldPointsOutsideField*0.005;

      if( confidence < 0.0 ) { confidence = 0.0; } // assure the subtrack cannot result in negative confidence
   } else {
      // not enough line points to determine the position
      confidence = 0.0;
   }

   // qDebug("INFO    linepoints %3d inside %3d outside %3d sum %5d confidence %6.3f",
   //       lPoints->linePointsSize, fieldPointsAmount, fieldPointsOutsideField, confidenceSum, confidence);


   scoreStruct retVal;
   retVal.score = 1.0 - confidence; // TODO remove score
   retVal.confidence = confidence;
   retVal.amountOnFloor = fieldPointsAmount;
   retVal.amountOffFloor = fieldPointsOutsideField;
   if( confidence > 0.7 ) {
      retVal.positionFound = true;
   } else {
      retVal.positionFound = false;
   }
   return retVal;
}
