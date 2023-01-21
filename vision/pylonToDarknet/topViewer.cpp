// Copyright 2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <QDebug>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QtMath>

#include "topViewer.hpp"
#include "mainWidget.hpp"
#include "ui_topViewer.h"

topViewer::topViewer(mainWidget *parent) :
   QDialog(parent),
   ui(new Ui::topViewer) {
   this->ui->setupUi(this);
   this->parent = parent;

   this->topScene = new QGraphicsScene();

   calcLast.age = 0;
   calcLast.confidence = 0.0;
   calcLast.goodEnough = false;
   calcLast.lastActive = INT32_MAX;
   calcLast.pos.x = 0.5;
   calcLast.pos.y = 0.5;
   calcLast.pos.rz = 0.0;

   this->setWindowTitle("top viewer");

   this->ui->topView->setScene(topScene);
   this->ui->topView->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
   this->ui->topView->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

   // TODO: use dimensions when the last time used (through stored config file)
   QRect myRect = this->geometry();
   this->setGeometry(QRect(myRect.left(), myRect.top(), FLOOR_WIDTH, FLOOR_HEIGHT));
   this->topScene->setBackgroundBrush(QBrush(Qt::darkGreen));

   penDotLine.setColor(Qt::lightGray);
   penDotLine.setStyle(Qt::DotLine);
}

topViewer::~topViewer() {
   delete this->ui;
}

void topViewer::setup(linePoints *lPoints, robotFloor *rFloor) {
   this->lPoints = lPoints;
   this->rfloor = rFloor;
}

// show the robot location and line points on the floor
void topViewer::update() {
   float topWidth = ui->topView->width();
   float topHeight = ui->topView->height();

   rfloor->update(topWidth, topHeight); // also clears the scene

   // show the position with the best confidence
   detPosSt calc = parent->localiztn->getCalc();
   if( calc.goodEnough ) {
      calcLast = calc;
   }

   // make robot and line points grey when no new position found
   QColor pixelColor = Qt::white;
   QColor robotColor = Qt::red;
   if( calc.lastActive > 2 ) { // accept 2 misses before changing the color
      pixelColor = Qt::lightGray;
      robotColor = Qt::lightGray;
   }

   addStatus(calc);

   float x = calcLast.pos.x;
   float y = calcLast.pos.y;
   float rz = calcLast.pos.rz;

   // draw the robot
   drawRobot(x, y, rz, robotColor);


   // draw the line points relative to the robot
   std::vector<azimuthRadiusSt> pixelList = parent->localiztn->getPixelList();
   for( size_t ii = 0; ii < pixelList.size(); ii++ ) {
      drawPixel(pixelList[ii], x, y, rz, pixelColor);
   }
}

// add text to the topViewer
void topViewer::addStatus(const detPosSt calc) {
   float x = 0.01;
   float y = 0.01;
   rfloor->addText(x, y, QString().asprintf("on floor %3d", calc.amountOnFloor));
   y += 0.02;
   rfloor->addText(x, y, QString().asprintf("off floor %d", calc.amountOffFloor));
   y += 0.02;
   rfloor->addText(x, y, QString().asprintf("tries %d", calc.numberOfTries));
   y += 0.02;
   rfloor->addText(x, y, QString().asprintf("age %d ", calc.age));
   y += 0.02;
   rfloor->addText(x, y, QString().asprintf("last active %d ", calc.lastActive));
   y += 0.02;
   rfloor->addText(x, y, QString().asprintf("confidence %3.0f %%", 100.0 * calc.confidence));
   y += 0.02;
   rfloor->addText(x, y, QString().asprintf("x %4.0f", 1000.0 * calc.pos.x));
   y += 0.02;
   rfloor->addText(x, y, QString().asprintf("y %4.0f", 1000.0 * calc.pos.y));
   y += 0.02;
   rfloor->addText(x, y, QString().asprintf("rz %4.0f", 360.0 * calc.pos.rz / ( 2.0 * M_PI ) ));
}

// draw the robot on the floor
void topViewer::drawRobot(const float x, const float y, const float rz, const QColor color) {
   float xScale = rfloor->getXScale();
   float yScale = rfloor->getYScale();

   // draw cirle
   float diameter = 0.5;
   rfloor->addEllipse(x, y, xScale * diameter, yScale * diameter, QPen(color), QBrush(Qt::transparent) );

   // draw shooter
   float length = 1.2 * diameter;
   float xBegin = x + xScale * diameter/2.0 * qSin(rz);
   float xEnd = x + xScale * length * qSin(rz);
   float yBegin = y - yScale * diameter/2.0 * qCos(rz);
   float yEnd = y - yScale * length * qCos(rz);

   rfloor->addLine(xBegin, yBegin, xEnd, yEnd, color);
}

// xOffset range 0.0 to 1.0 (for now from the xOffset slider)
// yOffset range 0.0 to 1.0 (for now from the yOffset slider)
// rZ range 0 to 2 Pi (for now from the rotate slider)
void topViewer::drawPixel(const azimuthRadiusSt object, const float xOffset, const float yOffset, const float rZ, const QColor color) {
   if ( ! object.valid ) {
      qDebug("ERROR   cannot draw invalid pixel x %4.2f y %4.2f rz %4.2f", xOffset, yOffset, rZ);
      return;
   }
   float xScale = rfloor->getXScale();
   float yScale = rfloor->getYScale();

   // azimuth definition: 0 degrees is north and 90 degrees is east
   float azimuth = object.azimuth + rZ;

   // enable next line to show all angles from robot to objects
   // rfloor->addLineAngle(xOffset, yOffset, azimuth, 0.3, 2, QPen(Qt::red));

   float xCenter = xOffset + xScale * object.radius * qSin(azimuth);
   float yCenter = yOffset - yScale * object.radius * qCos(azimuth);

   // determine if x or y is first out of range
   // the following combinations are possible
   // x >= 0.0 && x <= 1.0 && y >= 0.0 && y <= 1.0 : all in range
   // x >= 0.0 && x <= 1.0 && y < 0.0 : y out of range
   // x >= 0.0 && x <= 1.0 && y > 1.0 : y out of range
   // x < 0.0 && y >= 0.0 && y <= 1.0 : x out of range
   // x > 1.0 && y >= 0.0 && y <= 1.0 : x out of range
   // x > 1.0 && y > 1.0 : both out of range
   // x > 1.0 && y < 0.0 : both out of range
   // x < 0.0 && y > 1.0 : both out of range
   // x < 0.0 && y < 0.0 : both out of range

   if( xCenter >= 0.0 && xCenter <= 1.0 && yCenter >= 0.0 && yCenter <= 1.0 ) {
      // all in range, do nothing
   } else if( xCenter >= 0.0 && xCenter <= 1.0 && yCenter < 0.0 ) { // yCenter out of range
      xCenter = xOffset + (xScale/yScale) * yOffset * qTan(azimuth);
      yCenter = 0.0;
      // qDebug() <<"yCenter < 0.0 xCenter" << xCenter;
   } else if( xCenter >= 0.0 && xCenter <= 1.0 && yCenter > 1.0 ) { // yCenter out of range
      xCenter = xOffset - (xScale/yScale) * (1.0 - yOffset) * qTan(azimuth);
      yCenter = 1.0;
      // qDebug() <<"yCenter > 1.0 xCenter" << xCenter;
   } else if( xCenter < 0.0 && yCenter >= 0.0 && yCenter <= 1.0 ) { // xCenter out of range
      xCenter = 0.0;
      yCenter = yOffset + (yScale/xScale) *  xOffset / qTan(azimuth);
      // qDebug() <<"xCenter < 0.0 yCenter" << yCenter;
   } else if( xCenter > 1.0 && yCenter >= 0.0 && yCenter <= 1.0 ) { // xCenter out of range
      xCenter = 1.0;
      yCenter = yOffset - (yScale/xScale) * (1.0 - xOffset) / qTan(azimuth);
      // qDebug() <<"xCenter > 1.0 yCenter" << yCenter;
   } else if( xCenter > 1.0 && yCenter > 1.0 ) { // both out of range
      float xCenterTest = xOffset - (xScale/yScale) * (1.0 - yOffset) * qTan(azimuth);
      float yCenterTest = yOffset - (yScale/xScale) * (1.0 - xOffset) / qTan(azimuth);
      if( yCenterTest <= 1.0 ) { // xCenter first out of range
         xCenter = 1.0;
         yCenter = yCenterTest;
         // qDebug() <<"xCenter > 1.0 && yCenter > 1.0 yCenter" << yCenter;
      } else if( xCenterTest <= 1.0 ) { // yCenter first out of range
         xCenter = xCenterTest;
         yCenter = 1.0;
         // qDebug() <<"xCenter > 1.0 && yCenter > 1.0 xCenter" << xCenter;
      } else {
         qDebug() << "ERROR   xCenter > 1.0 && yCenter > 1.0 cannot determine border, xCenter" << xCenterTest << "yCenter" << yCenterTest;
      }
   } else if( xCenter > 1.0 && yCenter < 0.0 ) { // both out of range
      float xCenterTest = xOffset + (xScale/yScale) * yOffset * qTan(azimuth);
      float yCenterTest = yOffset - (yScale/xScale) * (1.0 - xOffset) / qTan(azimuth);
      if( yCenterTest >= 0.0 ) { // xCenter first out of range
         xCenter = 1.0;
         yCenter = yCenterTest;
         // qDebug() <<"xCenter > 1.0 && yCenter < 0.0 yCenter" << yCenter;
      } else if( xCenterTest <= 1.0 ) { // yCenter first out of range
         xCenter = xCenterTest;
         yCenter = 0.0;
         // qDebug() <<"xCenter > 1.0 && yCenter < 0.0 xCenter" << xCenter;
      } else {
         qDebug() << "ERROR   xCenter > 1.0 && yCenter < 0.0 cannot determine border, xCenter" << xCenterTest << "yCenter" << yCenterTest;
      }
   } else if( xCenter < 0.0 && yCenter < 0.0 ) { // both out of range
      float xCenterTest = xOffset + (xScale/yScale) * yOffset * qTan(azimuth);
      float yCenterTest = yOffset + (yScale/xScale) *  xOffset / qTan(azimuth);
      if( yCenterTest >= 0.0 ) { // xCenter first out of range
         xCenter = 0.0;
         yCenter = yCenterTest;
         // qDebug() <<"xCenter < 0.0 && yCenter < 0.0 yCenter" << yCenter;
      } else if( xCenterTest >= 0.0 ) { // yCenter first out of range
         xCenter = xCenterTest;
         yCenter = 0.0;
         // qDebug() <<"xCenter < 0.0 && yCenter < 0.0 xCenter" << xCenter;
      } else {
         qDebug() << "ERROR   xCenter < 0.0 && yCenter < 0.0 cannot determine border, xCenter" << xCenterTest << "yCenter" << yCenterTest;
      }
   } else if( xCenter < 0.0 && yCenter > 1.0 ) { // both out of range
      float xCenterTest = xOffset - (xScale/yScale) * (1.0 - yOffset) * qTan(azimuth);
      float yCenterTest = yOffset + (yScale/xScale) *  xOffset / qTan(azimuth);
      if( yCenterTest <= 1.0 ) { // xCenter first out of range
         xCenter = 0.0;
         yCenter = yCenterTest;
         // qDebug() <<"xCenter < 0.0 && yCenter > 1.0 yCenter" << yCenter;
      } else if( xCenterTest >= 0.0 ) { // yCenter first out of range
         xCenter = xCenterTest;
         yCenter = 1.0;
         // qDebug() <<"xCenter < 0.0 && yCenter > 1.0 xCenter" << xCenter;
      } else {
         qDebug() << "ERROR   xCenter < 0.0 && yCenter > 1.0 cannot determine border, xCenter" << xCenterTest << "yCenter" << yCenterTest;
      }
   } else {
      qDebug() << "ERROR   invalid xCenter" << xCenter << "yCenter" << yCenter << "combination";
   }

   if( xCenter < 0.0 || xCenter > 1.0 || yCenter < 0.0 || yCenter > 1.0 ) {
      qDebug() << "ERROR   xCenter" << xCenter << "or yCenter" << yCenter << "out of range";
   }

   // draw the line point on the floor
   QPen myPen(color);
   myPen.setWidth(4);
   rfloor->addPoint(xCenter, yCenter, myPen);
   // rfloor->addPoint(object.xFloor/(1000.0 * xScale), object.yFloor/(1000.0 * yScale), myPen);
}
