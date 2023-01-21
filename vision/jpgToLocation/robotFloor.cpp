// Copyright 2021-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <QDateTime>
#include <QDebug>
#include <QGraphicsRectItem>
#include <QGraphicsTextItem>
#include <QGraphicsScene>
#include <QtMath>
#include <QImage>
#include <QPainter>

#include "config.hpp"
#include "robotFloor.hpp"

robotFloor::robotFloor(QGraphicsScene *topScene) {
   this->topScene = topScene;

   float A                = 22.535; // field width including lines (x)
   float B                = 14.007; // field height including lines (y)
   float C                =  6.903; // penalty area height including lines (y)
   float D                =  3.902; // goal area height including lines (y)
   float E                =  2.376; // penaly araa width including lines (x)
   float F                =  0.765; // goal area width including lines (x)
   float G                =  0.760; // corner circle radius including lines
   float H                =  4.006; // inner circle diameter including lines
   float I                =  3.677; // penalty mark distance (x) including line and mark
   float J                =  0.153; // penalty mark diameter
   float K                =  0.174; // center mark diameter
   float goalWidth        =  2.649; // including goal posts
   float goalDepth        =  0.000; // 0.557 goalpost already marked as sunspot
   // float goalPostTichness =  0.125; // used by teamplay, not used by multiCam
   float lineThickness    =  0.133;
   float goalLineBorder   =  1.000; // exclude back side goal 0.760 # area outside white lines
   float touchLineBorder  =  1.000; // allow robot's to get a lock when out of the field

   // provided values are the outside of the lines, while we want the center of the lines
   // subtract half the with of the lines on both sides
   A -= lineThickness;
   B -= lineThickness;
   C -= lineThickness;
   D -= lineThickness;
   E -= lineThickness;
   F -= lineThickness;
   G -= lineThickness;
   H -= lineThickness;
   I -= lineThickness;
   J -= 0.0;
   K -= lineThickness/2.0 - J/2.0;


   goalWidth -= lineThickness; // goal width measured including both goal post
   goalDepth -= lineThickness/2.0; // measured including line
   if( goalDepth < 0.0 ) { goalDepth = 0.0; } // prevent a negative goalDepth in case the goal post "line" is not needed (and set to 0)

   goalLineBorder += lineThickness/2.0;
   touchLineBorder += lineThickness/2.0;

   xScale = 1.0 / (A + 2.0 * goalLineBorder);
   yScale = 1.0 / (B + 2.0 * touchLineBorder);

   floorSize.xLeft = 0.0;
   floorSize.xRight = 1.0;
   floorSize.yTop = 0.0;
   floorSize.yBottom = 1.0;

   fieldSize.xLeft = xScale * goalLineBorder;
   fieldSize.xMiddle = xScale * ( goalLineBorder + A/2.0);
   fieldSize.xRight = xScale * (goalLineBorder + A);
   fieldSize.yTop = yScale * touchLineBorder;
   fieldSize.yMiddle = yScale * (touchLineBorder + B/2.0);
   fieldSize.yBottom = yScale * (touchLineBorder + B);
   // qDebug() << fieldSize.xLeft << fieldSize.xRight << fieldSize.yTop <<fieldSize.yBottom;
   // qDebug() << fieldSize.xMiddle << fieldSize.yMiddle;

   penaltySize.xLeftArea.left = xScale * goalLineBorder;
   penaltySize.xLeftArea.right = xScale * (goalLineBorder + E);
   penaltySize.xLeftArea.mark = xScale * (goalLineBorder + I);
   penaltySize.xRightArea.left = xScale * (goalLineBorder + A - E);
   penaltySize.xRightArea.right = xScale * (goalLineBorder + A);
   penaltySize.xRightArea.mark = xScale * (goalLineBorder + A - I);
   penaltySize.yTop = yScale * (touchLineBorder + B/2.0 - C/2.0);
   penaltySize.yBottom = yScale * (touchLineBorder + B/2.0 + C/2.0);
   penaltySize.arc.xRadius = xScale * J/2.0;
   penaltySize.arc.yRadius = yScale * J/2.0;

   goalSize.xLeftArea.left = xScale * goalLineBorder;
   goalSize.xLeftArea.right = xScale * (goalLineBorder + F);
   goalSize.xRightArea.left = xScale * ( goalLineBorder + A - F);
   goalSize.xRightArea.right = xScale * ( goalLineBorder + A);
   goalSize.yTop = yScale * ( touchLineBorder + B/2.0 - D/2.0);
   goalSize.yBottom = yScale * ( touchLineBorder + B/2.0 + D/2.0);
   goalSize.xDepth = xScale * goalDepth; ; // to prevent wrong locking, APOX todo: remove
   goalSize.yPoleToCenter = goalWidth/2.0; // this one not from the spreadsheet but visually determined, APOX todo: remove (also part of the goal line)

   cornerArc.xRadius = xScale * G;
   cornerArc.yRadius = yScale * G;

   centerArc.xRadius = xScale * H/2.0;
   centerArc.yRadius = yScale * H/2.0;

   // create cost QImage (to be able to desplay on robot floor) and cost table (for position calculation)
   costImage = new QImage(FLOOR_WIDTH, FLOOR_HEIGHT, QImage::Format_RGB32); // QImage::Format_Grayscale8
   QPainter layout(costImage);

   // add all white lines and circles to floor
   createLayout(layout, true);

   // copy QImage to normal matrix (to speed up the generation of the cost table)
   for( int x = 0; x < FLOOR_WIDTH; x++ ) {
      for( int y = 0; y < FLOOR_HEIGHT; y++ ) {
         if( ( costImage->pixel(x,y) & 0x00ffffff ) != 0 ) {
            costTableIn[x][y] = 255; // pixel on white line
         } else {
            costTableIn[x][y]= 0;
         }
      }
   }

   blurFloor();

   // convert back the matrix to a QImage (to be able to display on robot floor)
   for( int x = 0; x < FLOOR_WIDTH; x++ ) {
      for( int y = 0; y < FLOOR_HEIGHT; y++ ) {
         quint8 pixel = costTable[x][y];
         QRgb value = 0xff000000 | pixel << 16 | pixel << 8 | pixel; // gray scale image
         costImage->setPixel(x, y, value);
      }
   }

   // the cost table is visible when not calling the update method
   topScene->setSceneRect(0,0,FLOOR_WIDTH, FLOOR_HEIGHT);
   QGraphicsPixmapItem* imageItem = new QGraphicsPixmapItem(QPixmap::fromImage(*costImage));
   topScene->addItem(imageItem);
}

void robotFloor::createLayout(QPainter &paint, const bool costTable) {
   QPen pen;
   if( costTable ) { // do use normal white lines for costTable
      topWidth = FLOOR_WIDTH; // do not scale cost table with display layout
      topHeight = FLOOR_HEIGHT;
      pen.setColor(Qt::white);
   } else {
      pen.setColor(Qt::lightGray);
      pen.setStyle(Qt::DotLine);
   }
   paint.setPen(pen);
   paint.setBrush(Qt::transparent);

   addRect(paint, fieldSize.xLeft,fieldSize.xRight,fieldSize.yTop,fieldSize.yBottom);
   addLine(paint, fieldSize.xMiddle, fieldSize.yTop, fieldSize.xMiddle, fieldSize.yBottom);

   addLine(paint, penaltySize.xLeftArea.left, penaltySize.yTop, penaltySize.xLeftArea.right, penaltySize.yTop);
   addLine(paint, penaltySize.xLeftArea.left, penaltySize.yBottom, penaltySize.xLeftArea.right, penaltySize.yBottom);
   addLine(paint, penaltySize.xLeftArea.right, penaltySize.yTop, penaltySize.xLeftArea.right, penaltySize.yBottom);
   addLine(paint, penaltySize.xRightArea.left, penaltySize.yTop, penaltySize.xRightArea.right, penaltySize.yTop);
   addLine(paint, penaltySize.xRightArea.left, penaltySize.yBottom, penaltySize.xRightArea.right, penaltySize.yBottom);
   addLine(paint, penaltySize.xRightArea.left, penaltySize.yTop, penaltySize.xRightArea.left, penaltySize.yBottom);

   addLine(paint, goalSize.xLeftArea.left, goalSize.yTop, goalSize.xLeftArea.right, goalSize.yTop);
   addLine(paint, goalSize.xLeftArea.left, goalSize.yBottom, goalSize.xLeftArea.right, goalSize.yBottom);
   addLine(paint, goalSize.xLeftArea.right, goalSize.yTop, goalSize.xLeftArea.right, goalSize.yBottom);

   addLine(paint, goalSize.xRightArea.left, goalSize.yTop, goalSize.xRightArea.right, goalSize.yTop);
   addLine(paint, goalSize.xRightArea.left, goalSize.yBottom, goalSize.xRightArea.right, goalSize.yBottom);
   addLine(paint, goalSize.xRightArea.left, goalSize.yTop, goalSize.xRightArea.left, goalSize.yBottom);

   paint.setBrush(Qt::lightGray);
   addEllipse(paint, penaltySize.xLeftArea.mark, fieldSize.yMiddle, 2.0*penaltySize.arc.xRadius, 2.0*penaltySize.arc.yRadius);
   addEllipse(paint, penaltySize.xRightArea.mark, fieldSize.yMiddle, 2.0*penaltySize.arc.xRadius, 2.0*penaltySize.arc.yRadius);
   paint.setBrush(Qt::transparent);

   addEllipse(paint, fieldSize.xMiddle, fieldSize.yMiddle, 2.0*centerArc.xRadius, 2.0* centerArc.yRadius);

   addArc(paint, fieldSize.xLeft, fieldSize.yTop, cornerArc.xRadius, cornerArc.yRadius, 270, 90);
   addArc(paint, fieldSize.xRight, fieldSize.yTop, cornerArc.xRadius, cornerArc.yRadius, 180, 90);
   addArc(paint, fieldSize.xRight, fieldSize.yBottom, cornerArc.xRadius, cornerArc.yRadius, 90, 90);
   addArc(paint, fieldSize.xLeft, fieldSize.yBottom, cornerArc.xRadius, cornerArc.yRadius, 0, 90);
}

void robotFloor::update(quint16 width, quint16 height) {
   float ratio = width * xScale / (height * yScale);
   if ( ratio < 1.0 ) {
      // height is relative larger then width
      // use width as max value and downscale height;
      topWidth = width;
      topHeight = height * ratio;
   } else {
      topWidth = width / ratio;
      topHeight = height;
   }

   topScene->clear();
   topScene->setSceneRect(0,0,topWidth, topHeight);
   QImage floor(topWidth, topHeight, QImage::Format_RGB32);
   floor.fill(0x006400);
   QPainter layout(&floor);
   createLayout(layout, false);
   QGraphicsPixmapItem* imageItem = new QGraphicsPixmapItem(QPixmap::fromImage(floor));
   topScene->addItem(imageItem);
}

void robotFloor::addText(const float x, const float y, const QString text, const int size, const QColor color) {
   QFont myFont;
   myFont.setPixelSize(size);

   QGraphicsTextItem *textItem = topScene->addText(text);
   textItem->setDefaultTextColor(color);
   textItem->setFont(myFont);
   textItem->setPos(x * topWidth, y * topHeight); // scale from relative to pixels
}

void robotFloor::addTopViewText(quint64 captureTime) {
   QFont myFont;
   myFont.setPixelSize(12);

   // show frame counter, date and time in the top viewer
   QDateTime datetime;
   datetime.setMSecsSinceEpoch(captureTime); // provided in milliseconds

   QString line;
   line.append(datetime.toString("HH:mm:ss.zzz"));
   line.chop(1); // change from 1/1000 sec to 1/100 sec

   QGraphicsTextItem *text = topScene->addText(line);
   text->setDefaultTextColor(Qt::white);
   text->setFont(myFont);
   text->setPos(4,4);
}

void robotFloor::addEllipse(QPainter &paint, const float xCenter, const float yCenter, const float xRadius, const float yRadius) {
   float xLeftPixel = ( xCenter - xRadius/2.0 ) * topWidth;
   float yTopPixel = ( yCenter - yRadius/2.0 ) * topHeight;
   float widthPixel = xRadius * topWidth;
   float heightPixel = yRadius * topHeight;
   paint.drawEllipse(xLeftPixel, yTopPixel, widthPixel, heightPixel);
}

void robotFloor::addEllipse(const float xCenter, const float yCenter, const float xRadius, const float yRadius, const QPen pen = QPen(Qt::red), const QBrush brush = Qt::transparent) {
   float xLeftPixel = ( xCenter - xRadius/2.0 ) * topWidth;
   float yTopPixel = ( yCenter - yRadius/2.0 ) * topHeight;
   float widthPixel = xRadius * topWidth;
   float heightPixel = yRadius * topHeight;
   topScene->addEllipse(xLeftPixel, yTopPixel, widthPixel, heightPixel, pen, brush);
}

void robotFloor::addLine(QPainter &paint, const float x0, const float y0, const float x1, const float y1) {
   paint.drawLine(x0 * topWidth, y0 * topHeight, x1 * topWidth, y1 * topHeight);
}

void robotFloor::addLine(const float x0, const float y0, const float x1, const float y1, const QPen pen) {
   topScene->addLine(x0 * topWidth, y0 * topHeight, x1 * topWidth, y1 * topHeight, pen);
}

void robotFloor::addPoint(const float x, const float y, const QPen pen) {
   topScene->addLine(x * topWidth, y * topHeight, x * topWidth, y * topHeight, pen);
}

// radius in meters
void robotFloor::addLineAngle(QPainter &paint, const float x, const float y, const float angle, const float radius0, const float radius1) {
   float x0 = x + xScale * radius0 * qSin(angle);
   float y0 = y - yScale * radius0 * qCos(angle);
   float x1 = x + xScale * radius1 * qSin(angle);
   float y1 = y - yScale * radius1 * qCos(angle);
   // qDebug() << "sin" << qSin(angle) << "cos" << qCos(angle) << "angle" << 180.0 * angle / M_PI;

   paint.drawLine(x0 * topWidth, y0 * topHeight, x1 * topWidth, y1 * topHeight);
}

void robotFloor::addRect(QPainter &paint, const float xLeft, const float xRight, const float yTop, const float yBottom) {
   float xLeftPixel = xLeft * topWidth;
   float yTopPixel = yTop * topHeight;
   float widthPixel = ( xRight - xLeft ) * topWidth;
   float heightPixel = ( yBottom - yTop ) * topHeight;

   paint.drawRect(xLeftPixel, yTopPixel, widthPixel, heightPixel);
}

void robotFloor::addArc(QPainter &paint, const float xCenter, const float yCenter, const float xRadius, const float yRadius, const float angleStart, const float angleDelta) {
   float xLeftPixel = (xCenter - xRadius) * topWidth;
   float yTopPixel = (yCenter - yRadius) * topHeight;
   float widthPixel = 2 * xRadius* topWidth;
   float heightPixel = 2 * yRadius * topHeight;

   QPainterPath path;
   path.arcMoveTo(xLeftPixel,yTopPixel,widthPixel,heightPixel, angleStart );
   path.arcTo(xLeftPixel,yTopPixel,widthPixel,heightPixel, angleStart, angleDelta);
   paint.drawPath(path);
}

// TODO: the blur function has a wide area with value 255, try to make it a bit more linear

// convert the floor into a blurred floor
// this is done by calculating for each pixel the values of the surrounding pixels
// because the surrounding also have surrounding, the calculation is run iterative
// this calculation is not part of the main calculation loop, so optimization is not
// that important for this function (e.g. using a memcpy for the table copy)
void robotFloor::blurFloor() {
   int blurPixels = 80;
   for( int ii = 0; ii < blurPixels; ii++ ) {
      float blurValue = cos( 2.0*M_PI*ii/(4.0*blurPixels) ); // use only first 45 degrees of cosine curve
      blurPixel(blurValue);
      for( int x = 0; x < FLOOR_WIDTH; x++ ) {
         for( int y = 0; y < FLOOR_HEIGHT; y++ ) {
            costTableIn[x][y] = costTable[x][y];
         }
      }
   }
}

// copy each pixel to a new floor and copy also the 8 neighbours to the new floor
// the 8 neighbours should have a lower value then the center pixel
void robotFloor::blurPixel(const float blurValue) {
   for( int x = 1; x < FLOOR_WIDTH - 1; x++ ) { // reduce range to prevent out of range access
      for( int y = 1; y < FLOOR_HEIGHT - 1; y++ ) { // reduce range to prevent out of range access
         quint8 value = costTableIn[x][y];
         if( costTableIn[x][y] < value ) { costTable[x][y] = value; } // center pixel
         value = (quint8)(blurValue * (float)value + 0.5f); // calculate the lower value for the 8 surrounding pixels
         if( costTableIn[x-1][y-1] < value ) { costTable[x-1][y-1] = value; } // bottom left
         if( costTableIn[x  ][y-1] < value ) { costTable[x  ][y-1] = value; } // bottom
         if( costTableIn[x+1][y-1] < value ) { costTable[x+1][y-1] = value; } // bottom right
         if( costTableIn[x+1][y  ] < value ) { costTable[x+1][y  ] = value; } // right
         if( costTableIn[x+1][y+1] < value ) { costTable[x+1][y+1] = value; } // top right
         if( costTableIn[x  ][y+1] < value ) { costTable[x  ][y+1] = value; } // top
         if( costTableIn[x-1][y+1] < value ) { costTable[x-1][y+1] = value; } // top left
         if( costTableIn[x-1][y  ] < value ) { costTable[x-1][y  ] = value; } // left
      }
   }
}
