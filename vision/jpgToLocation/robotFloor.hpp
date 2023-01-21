// Copyright 2021-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef ROBOTFLOOR_HPP
#define ROBOTFLOOR_HPP

#include <QGraphicsScene>
#include <QImage>
#include <QPainter>

#include "config.hpp"

class robotFloor
{
public:
   robotFloor(QGraphicsScene *topScene);
   // void draw();
   void update(quint16 width, quint16 height);
   void addEllipse(const float xCenter, const float yCenter, const float xRadius, const float yRadius, const QPen pen, QBrush brush);
   void addLine(const float x0, const float y0, const float x1, const float y1, const QPen pen);
   void addPoint(const float x, const float y, const QPen pen);
   quint8 costTable[FLOOR_WIDTH][FLOOR_HEIGHT]; // for maximal performance, direct access to lookup table by search algorithm

   float getXScale() { return xScale; } // typically around ~= 0.0513
   float getYScale() { return yScale; } // typically around ~= 0.0714
   void addTopViewText(quint64 captureTime);
   void addText(const float x, const float y, const QString text, const int size = 12, const QColor color = Qt::white);

   typedef struct {
      float xLeft;
      float xMiddle;
      float xRight;
      float yTop;
      float yMiddle;
      float yBottom;
   } fieldSizeStruct;

   fieldSizeStruct getFieldSize() { return fieldSize; } // used to check ranges

private:

   void addEllipse(QPainter &paint, const float xCenter, const float yCenter, const float width, const float height);
   void addRect(QPainter &paint, const float xLeft, const float xRight, const float yTop, const float yBottom);
   void addLine(QPainter &paint, const float x0, const float y0, const float x1, const float y1);
   void addLineAngle(QPainter &paint, const float x, const float y, const float angle, const float radius0, const float radius1);
   void addArc(QPainter &paint, const float xBegin, const float yBegin, const float xRadius, const float yRadius, const float angleStart, const float angleDelta);
   void createLayout(QPainter &paint, const bool costTable);

   void blurFloor();
   void blurPixel(const float blurValue);

   typedef struct {
      float xLeft;
      float xRight;
      float yTop;
      float yBottom;
   } floorSizeSt;

   typedef struct {
      float left;
      float right;
      float mark;
   } penaltyXSizeStruct;

   typedef struct {
      float xRadius;
      float yRadius;
   } arcStruct;

   typedef struct {
      penaltyXSizeStruct xLeftArea;
      penaltyXSizeStruct xRightArea;
      float yTop;
      float yBottom;
      arcStruct arc;
   } penaltySizeStruct;

   typedef struct {
      float left;
      float right;
   } goalXSizeStruct;

   typedef struct {
      goalXSizeStruct xLeftArea;
      goalXSizeStruct xRightArea;
      float yTop;
      float yBottom;
      float xDepth; // APOX todo: remove
      float yPoleToCenter; // APOX todo: remove
   } goalSizeStruct;

   float topWidth, topHeight;
   QGraphicsScene *topScene = nullptr;
   floorSizeSt floorSize;
   fieldSizeStruct fieldSize;
   penaltySizeStruct penaltySize;
   goalSizeStruct goalSize;
   arcStruct cornerArc;
   arcStruct centerArc;
   float xScale;
   float yScale;

   QImage *costImage;
   quint8 costTableIn[FLOOR_WIDTH][FLOOR_HEIGHT];
};

#endif // ROBOTFLOOR_HPP
