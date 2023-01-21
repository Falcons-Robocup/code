// Copyright 2021-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <QDebug>
#include <QtMath>

#include <algorithm> // std::random_shuffle

#include "linePoints.hpp"

linePoints::linePoints(const int robot) {
   linePointsSize = 0;
   this->robot = robot;

   for( size_t camera = 0; camera < CAMS; camera++ ) {
      floorImage[camera] = new QImage(CAM_WIDTH, CAM_HEIGHT, QImage::Format_Mono);
      lineImage[camera] = new QImage(CAM_WIDTH, CAM_HEIGHT, QImage::Format_Mono);

      dewarpLookup[camera] = new Dewarper(robot, camera, false);
      if( dewarpLookup[camera]->getPixelWith() != CAM_WIDTH ) {
         qDebug() << "ERROR   cam" << camera << "width of" << CAM_WIDTH << "does not match with dewarp table width of" << dewarpLookup[camera]->getPixelWith();
      }
      if( dewarpLookup[camera]->getPixelHeight() != CAM_HEIGHT ) {
         qDebug() << "ERROR   cam" << camera << "height of" << CAM_HEIGHT << "does not match with dewarp table height of" << dewarpLookup[camera]->getPixelHeight();
      }
   }

   rasterSize = 20;
   createRasterImage();
}

void linePoints::update(QImage *camImage[CAMS]) {
   // create line points from the camera images
   linePointsList.clear();
   for( size_t camera = 0; camera < CAMS; camera++ ) {
      filterImage(camImage[camera], camera); // create floorImage and lineImage

      // use the floorImage and lineImage to create a list with line points
      camLinePoints[camera] = scanLines(camera, true); // scan for vertical lines (for portrait camera layout)
      camLinePoints[camera].append(scanLines(camera, false)); // scan for horizontal lines (for portrait camera layout)

      // dewarp the line points and add to linePointList
      for( int ii = 0; ii < camLinePoints[camera].size(); ii++ ) {
         azimuthRadiusSt polar = cartesianToPolar(camera, ii);
         if( polar.valid ) { // not all input pixels can be dewarped
            // azimuth definition: 0 degrees is north and 90 degrees is east
            polar.azimuth = polar.azimuth + camera * M_PI/2.0; // camera's are placed with 90 degrees offset
            linePointsList.push_back(polar); // collect in list to be used by determinePosition
         }
      }
   }

   // limit the amount of found line pixels
   linePointsSize = linePointsList.size();
   if( linePointsSize > LINE_POINTS_NUMBER_MAXIMAL ) {
      linePointsSize = LINE_POINTS_NUMBER_MAXIMAL;
   }

   // when more line points are available then needed, randomize the ones used
   std::random_shuffle(linePointsList.begin(), linePointsList.end());

   // use the first amount (LINE_POINTS_NUMBER_MAXIMAL) from randomized list
   for( int ii = 0; ii < linePointsSize; ii++ ) {
      array[ii].azimuth = linePointsList[ii].azimuth;
      array[ii].radius = linePointsList[ii].radius;
   }
}

// create floorImage (for the green floor pixels) and lineImage (for the white line pixels) from
// the camera image
void linePoints::filterImage(const QImage *image, const size_t camera) {
   for( int yy = 0; yy < image->height(); yy++ ) {
      for( int xx = 0; xx < image->width(); xx++ ) {
         QPoint pos(xx, yy);

         floorImage[camera]->setPixel(pos, 0);
         lineImage[camera]->setPixel(pos, 0);

         // TODO: determine if hsl makes more sense then hsv
         QColor hsvPixel = QColor(image->pixel(pos)).toHsv();

         int hue = hsvPixel.hue(); // WARNING: range [0:359] and -1 for achromatic color (white, grey, black)
         int saturation = hsvPixel.saturation();
         int value = hsvPixel.value();

         if( hue < -1 || hue > 359 || saturation < 0 || saturation > 255 || value < 0 || value > 255 ) {
            qDebug("ERROR   hue %4d or saturation %4d or value %4d out of range", hue, saturation, value);
         }

         if( hue >= hueMin[0] && hue <= hueMax[0] &&
             saturation >= satMin[0] && saturation <= satMax[0] &&
             value >= valMin[0] && value <= valMax[0] ) {
            // pixel is within range for green
            floorImage[camera]->setPixel(pos, 1); // floor.pixel for Format_Mono is 0xff000000 (0) or 0xffffffff (1)
         } else if( ( hue == -1 || ( hue >= hueMin[1] && hue <= hueMax[1] ) ) &&
                    saturation >= satMin[1] && saturation <= satMax[1] &&
                    value >= valMin[1] && value <= valMax[1] ) {
            // pixel is within range for white
            lineImage[camera]->setPixel(pos, 1); // lineImage.pixel for Format_Mono is 0xff000000 (0) or 0xffffffff (1)
         }
      }
   }

   // *lineImage[camera] = *rasterImage; // for test, show raster in lineScene
}

void linePoints::createRasterImage() {
   rasterImage = new QImage(CAM_WIDTH, CAM_HEIGHT, QImage::Format_Mono);
   for( int yy = 0; yy < CAM_HEIGHT; yy++ ) {
      for( int xx = 0; xx < CAM_WIDTH; xx++ ) {
         QPoint pos(xx, yy);
         rasterImage->setPixel(pos, 0);

         // double width, so it is visible when viewer scaling is 50%
         if( ( yy % rasterSize == rasterSize/2 || ( yy % rasterSize + 1 )== rasterSize/2 ) && yy > CAM_HEIGHT/2 ) {
            rasterImage->setPixel(pos, 1);
         }
         if( ( xx % rasterSize == rasterSize/2 || ( xx % rasterSize + 1 ) == rasterSize/2 ) && yy > CAM_HEIGHT/2 ) {
            rasterImage->setPixel(pos, 1);
         }
      }
   }
}

QVector<linePointSt> linePoints::scanLines(const size_t camera, const bool vertical) {
   // NOTE: line means white line
   // NOTE: floor means green floor

   enum state_t { INIT, FLOOR, LINE };
   state_t state;
   QVector<linePointSt> linePoints;
   size_t aaEnd;
   size_t bbEnd;

   // scanning needs be done in as vertical as horizontal direction
   if( vertical ) { // scan from top (y = 0) to bottom
      aaEnd = CAM_WIDTH; // row or collum index = x
      bbEnd = CAM_HEIGHT; // pixels = y
   } else { // scan from left (x = 0) to right
      aaEnd = CAM_HEIGHT; // row or collum index = y
      bbEnd = CAM_WIDTH; // pixels = x
   }

   for( size_t aa = rasterSize/2; aa < aaEnd; aa = aa + rasterSize ) { // row or collum index
      state = INIT;
      int floorCount = 0;
      int lineCount = 0;
      linePointSt linePoint;
      bool lineFound = false;

      for( size_t bb = 0; bb < bbEnd; bb++ ) { // pixels
         QPoint pos;
         if( vertical ) { // scan from top (y = 0) to bottom
            pos.setX(aa);
            pos.setY(bb);
         } else { // scan from left (x = 0) to right
            pos.setX(bb);
            pos.setY(aa);
         }
         bool floorPixel = floorImage[camera]->pixel(pos) == 0xffffffff;
         bool linePixel = lineImage[camera]->pixel(pos) == 0xffffffff;
         // search for floor pixels, at least 7 out of 10 pixels
         // search for line pixels, 3 out of 5
         // search for floor pixels, at least 7 out of 10 pixels
         // determine center of line pixels
         if( floorPixel ) {
            if( floorCount < 10 ) { // truncate to prevent sticking to long on not existing floor
               floorCount++;
            }
         } else if( floorCount > 0 ) {
            floorCount = floorCount - floorDec; // use more then -1 to prevent sticking to long on not existing floor
            if( floorCount < 0 ) {
               floorCount = 0;
            }
         }

         if( linePixel ) {
            if( lineCount < 10 ) { // truncate to prevent sticking to long on not existing line
               lineCount++;
            }
         } else if( lineCount > 0 ) {
            lineCount = lineCount - lineDec; // use more then -1 to prevent sticking to long on not existing floor
            if( lineCount < 0 ) {
               lineCount = 0;
            }
         }

         if( state == INIT ) { // not on the floor, search for the first floor pixel
            lineFound = false;
            if( floorCount > 0 ) {
               state = FLOOR;
            }
         } else if( state == FLOOR ) { // get confidence this is the floor
            if( floorCount <= 0 ) {
               state = INIT; // not on floor, restart search
               if( lineFound ) {
                  // qDebug("lost");
               }
            } else if( floorCount >= floorMinWidth ) { // are on the floor
               // qDebug() << "floorCount" << floorCount << "lineCount" << lineCount << "lineFound" << lineFound << "state" << state << "pos" << pos << "linePixel" << hex << linePixel;
               // likely we are on the floor
               if( lineFound ) { // pending line and high floor confidence = valid line -> store the center of the line
                  linePoints.append(linePoint);
                  // qDebug("found point");
                  lineFound = false;
               }
               if( linePixel ) { // on the floor and one or more line pixels, determine if this is a line or just noise
                  linePoint.begin = pos; // store begin position of line
                  linePoint.end = pos; // store the end position of line (this might already be the last line pixel)
                  lineFound = false; // threshold: line is only valid if enough line pixels
                  if( lineMinWidth == 1 ) { // for far away lines, there is only pixel to decide on
                     lineFound = true;
                  }
                  state = LINE; // start searching for line pixels
               }
            }
         } else if( state == LINE ) { // get confidence this is a line
            if( linePixel ) {
               linePoint.end = pos; // keep moving the end position as long as we see line pixels
            }
            int width = linePoint.end.x() - linePoint.begin.x() + linePoint.end.y() - linePoint.begin.y();
            if( width >= lineMinWidth ) { // above threshold, so very likely a line
               lineFound = true;
               // qDebug("line found");
            }

            if( lineCount <= 0 ) { // we are definitely not on a line anymore
               state = FLOOR; // NOTE: the lineCount threshold should be enough to cover the "grey" pixels between line and floor, so floorCount should now be greater then 0
            }
         }
      }
   }
   // qDebug() << linePoints;
   return linePoints;
}

// convert line points parallel on short axis from Cartesian to Polar
// radius in meters
// azimuth in -Pi to Pi
azimuthRadiusSt linePoints::cartesianToPolar(const int camera, const int linePointIndex) {
   azimuthRadiusSt polar;
   polar.valid = false;
   if( ! inRange(camera, linePointIndex) ) {
      return polar;
   }

   int xBegin = camLinePoints[camera][linePointIndex].begin.x();
   int xEnd = camLinePoints[camera][linePointIndex].end.x();
   int yBegin = camLinePoints[camera][linePointIndex].begin.y();
   int yEnd = camLinePoints[camera][linePointIndex].end.y();

   int16_t xCamera = (xBegin + xEnd) / 2;
   int16_t yCamera = (yBegin + yEnd) / 2;
   if( xCamera <= 0 || xCamera >= CAM_WIDTH ) {
      qDebug("ERROR   xCamera %3d out of range", xCamera);
      exit(EXIT_FAILURE);
   }
   if( yCamera <= 0 || yCamera >= CAM_HEIGHT ) {
      qDebug("ERROR   yCamera %3d out of range", yCamera);
      exit(EXIT_FAILURE);
   }

   // convert from camera dimensions (pixels) to field dimensions (meters)
   // y = 0 is at top of image, but transformFront expects y = 0 at bottom of image
   int16_t yCameraSwap = CAM_HEIGHT - 1 - yCamera;
   int16_t xFloor, yFloor; // output of transformFloor function
   if ( ! dewarpLookup[camera]->transformFloor(yCameraSwap, xCamera, yFloor, xFloor) ) { // closest by pixel, use this yField to calculate azimuth
      // qDebug("ERROR   dewarp lookup failed for x %3d y %3d", xCamera, yCameraSwap); // TODO enable and check if the reject makes sense for the x and y input
      return polar;
   }

   // TODO: checkout why azimuth needs to be set negative over here
   // would expect the x and y would have the correct polarity and atan2 should then also be correct
   // update: likely because y is from top to bottom instead of bottom to top
   if( robot > 8 ) {
      polar.azimuth = atan2(yFloor, xFloor);
   } else {
      // the camera index is clockwise on robot 9
      polar.azimuth = -atan2(yFloor, xFloor);
   }
   polar.radius = sqrt(xFloor * xFloor + yFloor * yFloor) / 1000.0; // convert to meters (dewarp is in millimeters)
   polar.xCamera = xCamera;
   polar.yCamera = yCamera;
   polar.xFloor = xFloor; // TODO: check if polarity swap required, where are xFloor and yFloor used?
   polar.yFloor = yFloor;
   polar.valid = true;

   return polar;
}

bool linePoints::inRange(const size_t camera, const size_t index) {
   int xBegin = camLinePoints[camera][index].begin.x();
   int xEnd = camLinePoints[camera][index].end.x();
   int yBegin = camLinePoints[camera][index].begin.y();
   int yEnd = camLinePoints[camera][index].end.y();

   // a pixel cannot be found on one of the corners (and 0,0) or outside the camera
   if( xBegin <= 0 || xEnd >= CAM_WIDTH-1 ) {
      qDebug() << "ERROR   x begin" << xBegin << "or x end" << xEnd << "out of range";
      return false;
   }

   if( yBegin <= 0 || yEnd >= CAM_HEIGHT - 1 ) {
      qDebug() << "ERROR   y begin" << yBegin << "or y end" << yEnd << "out of range";
      return false;
   }

   if( xBegin > xEnd ) {
      qDebug() << "ERROR  x begin" << xBegin << "cannot be the larger as x end" << xEnd;
      return false;
   }

   if( yBegin > yEnd ) {
      qDebug() << "ERROR  y begin" << yBegin << "cannot be the larger as y end" << yEnd;
      return false;
   }

   // UPDATE: pixels of 1 by 1 are also valid
   //    if( xBegin == xEnd && yBegin == yEnd ) {
   //        qDebug() << "ERROR   only one set can be the same x begin" << xBegin << "x end" << xEnd << "y begin" << yBegin << "y end" << yEnd;
   //        return false;
   //    }

   // check if pixel in on the floor, which is from ~camHeight/2 to camHeight
   if( yBegin < (CAM_HEIGHT/2-50) || yEnd < (CAM_HEIGHT/2-50) ) { // TODO: make configurable per camera
      // qDebug("WARNING y begin %3d y end %3d is not on the floor", yBegin ,yEnd);
      return false;
   }

   // skip the real nearby pixels, which might be reflections from the robot
   if( yBegin > CAM_HEIGHT-10 || yEnd > CAM_HEIGHT-10 ) { // TODO: make configurable per camera
      // qDebug("WARNING y begin %3d y end %3d is too close to the robot", yBegin, yEnd);
      return false;
   }
   return true;
}
