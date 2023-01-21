// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "robotFloor.hpp"

#include <QDateTime>
#include <QDebug>
#include <QGraphicsRectItem>
#include <QGraphicsTextItem>
#include <QtMath>
#include <QVector>

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
   // qDebug() << xScale << yScale;

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

   // qDebug() << ratio << topWidth << topHeight;

   topScene->clear();

   // scale scene to top scene area
   topScene->setSceneRect(0,0,width, height);

   QPen penGrid(Qt::lightGray);
   penGrid.setStyle(Qt::DotLine);
   QPen penYAxis(Qt::white);
   QPen penRed(Qt::red);

   addRectRel(fieldSize.xLeft,fieldSize.xRight,fieldSize.yTop,fieldSize.yBottom, penGrid, QBrush(Qt::transparent));

   addLineRel(fieldSize.xMiddle, fieldSize.yTop, fieldSize.xMiddle, fieldSize.yBottom, penGrid);

   // fieldSize.xMiddle = xScale * ( goalLineBorder + A/2.0);
   //fieldSize.yMiddle = yScale * (touchLineBorder + B/2.0);

   addLineRel(penaltySize.xLeftArea.left, penaltySize.yTop, penaltySize.xLeftArea.right, penaltySize.yTop, penGrid);
   addLineRel(penaltySize.xLeftArea.left, penaltySize.yBottom, penaltySize.xLeftArea.right, penaltySize.yBottom, penGrid);
   addLineRel(penaltySize.xLeftArea.right, penaltySize.yTop, penaltySize.xLeftArea.right, penaltySize.yBottom, penGrid);
   addLineRel(penaltySize.xRightArea.left, penaltySize.yTop, penaltySize.xRightArea.right, penaltySize.yTop, penGrid);
   addLineRel(penaltySize.xRightArea.left, penaltySize.yBottom, penaltySize.xRightArea.right, penaltySize.yBottom, penGrid);
   addLineRel(penaltySize.xRightArea.left, penaltySize.yTop, penaltySize.xRightArea.left, penaltySize.yBottom, penGrid);

   addLineRel(goalSize.xLeftArea.left, goalSize.yTop, goalSize.xLeftArea.right, goalSize.yTop, penGrid);
   addLineRel(goalSize.xLeftArea.left, goalSize.yBottom, goalSize.xLeftArea.right, goalSize.yBottom, penGrid);
   addLineRel(goalSize.xLeftArea.right, goalSize.yTop, goalSize.xLeftArea.right, goalSize.yBottom, penGrid);

   addLineRel(goalSize.xRightArea.left, goalSize.yTop, goalSize.xRightArea.right, goalSize.yTop, penGrid);
   addLineRel(goalSize.xRightArea.left, goalSize.yBottom, goalSize.xRightArea.right, goalSize.yBottom, penGrid);
   addLineRel(goalSize.xRightArea.left, goalSize.yTop, goalSize.xRightArea.left, goalSize.yBottom, penGrid);

   addEllipseRelCenter(penaltySize.xLeftArea.mark, fieldSize.yMiddle, 2.0*penaltySize.arc.xRadius, 2.0*penaltySize.arc.yRadius, QPen(Qt::lightGray), QBrush(Qt::lightGray));
   addEllipseRelCenter(penaltySize.xRightArea.mark, fieldSize.yMiddle, 2.0*penaltySize.arc.xRadius, 2.0*penaltySize.arc.yRadius, QPen(Qt::lightGray), QBrush(Qt::lightGray));

   addEllipseRelCenter(fieldSize.xMiddle, fieldSize.yMiddle, 2.0*centerArc.xRadius, 2.0* centerArc.yRadius, penGrid, QBrush(Qt::transparent));

   addArcRel(fieldSize.xLeft, fieldSize.yTop, cornerArc.xRadius, cornerArc.yRadius, 270, 90, penGrid);
   addArcRel(fieldSize.xRight, fieldSize.yTop, cornerArc.xRadius, cornerArc.yRadius, 180, 90, penGrid);
   addArcRel(fieldSize.xRight, fieldSize.yBottom, cornerArc.xRadius, cornerArc.yRadius, 90, 90, penGrid);
   addArcRel(fieldSize.xLeft, fieldSize.yBottom, cornerArc.xRadius, cornerArc.yRadius, 0, 90, penGrid);

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

void robotFloor::addEllipse(const float xCenter, const float yCenter, const float xRadius, const float yRadius, QPen pen, QBrush brush) {
   addEllipseRelCenter(xCenter, yCenter, xRadius, yRadius, pen, brush);
}

void robotFloor::addEllipseRelCenter(const float xCenter, const float yCenter, const float width, const float height, const QPen pen, const QBrush brush) {
   float xLeftPixel = ( xCenter - width/2.0 ) * topWidth;
   float yTopPixel = ( yCenter - height/2.0 ) * topHeight;
   float widthPixel = width * topWidth;
   float heightPixel = height * topHeight;

   topScene->addEllipse(xLeftPixel, yTopPixel, widthPixel, heightPixel, pen, brush);
}


void robotFloor::addLineRel(const float x0, const float y0, const float x1, const float y1, const QPen pen) {
   topScene->addLine(x0 * topWidth, y0 * topHeight, x1 * topWidth, y1 * topHeight, pen);
}

void robotFloor::addLine(const float x0, const float y0, const float x1, const float y1, QPen pen) {
   addLineRel(x0, y0, x1, y1, pen);
}

// radius in meters
void robotFloor::addLineAngle(const float x, const float y, const float angle, const float radius0, const float radius1, QPen pen) {
   float x0 = x + xScale * radius0 * qSin(angle);
   float y0 = y - yScale * radius0 * qCos(angle);
   float x1 = x + xScale * radius1 * qSin(angle);
   float y1 = y - yScale * radius1 * qCos(angle);
   // qDebug() << "sin" << qSin(angle) << "cos" << qCos(angle) << "angle" << 180.0 * angle / M_PI;
   addLineRel(x0, y0, x1, y1, pen);
}

void robotFloor::addRect(const float xCenter, const float yCenter, const float width, const float height, QPen pen, QBrush brush) {
   addRectRelCenter(xCenter, yCenter, width, height, pen, brush);
}

void robotFloor::addRectRelCenter(const float xCenter, const float yCenter, const float width, const float height, const QPen pen, const QBrush brush) {
   float xLeftPixel = ( xCenter - width / 2.0 ) * topWidth;
   float yTopPixel = ( yCenter - height / 2.0 ) * topHeight;
   float widthPixel = width * topWidth;
   float heightPixel = height * topHeight;

   topScene->addRect(xLeftPixel, yTopPixel, widthPixel, heightPixel, pen, brush);
}

void robotFloor::addRectRel(const float xLeft, const float xRight, const float yTop, const float yBottom, QPen pen, const QBrush brush) {
   float xLeftPixel = xLeft * topWidth;
   float yTopPixel = yTop * topHeight;
   float widthPixel = ( xRight - xLeft ) * topWidth;
   float heightPixel = ( yBottom - yTop ) * topHeight;

   topScene->addRect(xLeftPixel, yTopPixel, widthPixel, heightPixel, pen, brush);
}

void robotFloor::addArcRel(const float xCenter, const float yCenter, const float xRadius, const float yRadius, const float angleStart, const float angleDelta, const QPen pen) {
   float xLeftPixel = (xCenter - xRadius) * topWidth;
   float yTopPixel = (yCenter - yRadius) * topHeight;
   float widthPixel = 2 * xRadius* topWidth;
   float heightPixel = 2 * yRadius * topHeight;

   QPainterPath path;
   path.arcMoveTo(xLeftPixel,yTopPixel,widthPixel,heightPixel, angleStart );
   path.arcTo(xLeftPixel,yTopPixel,widthPixel,heightPixel, angleStart, angleDelta);
   topScene->addPath(path, pen);
}
