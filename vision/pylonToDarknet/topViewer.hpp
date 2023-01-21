// Copyright 2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef TOPVIEWER_HPP
#define TOPVIEWER_HPP

#include <QDialog>
#include <QGraphicsScene>
#include <QPen>

#include "config.hpp"
#include "determinePosition.hpp"
#include "robotFloor.hpp"

class mainWidget; //forward declaration

namespace Ui {
class topViewer;
}

class topViewer : public QDialog {
   Q_OBJECT

public:
   explicit topViewer(mainWidget *parent = nullptr);
   ~topViewer();
   void setup(linePoints *lPoints, robotFloor *rFloor);
   void update();

   QGraphicsScene *topScene = nullptr; // used by robotfloor


private:
   void addStatus(const detPosSt calc);
   void drawRobot(const float x, const float y, const float rz, const QColor color = Qt::red);
   void drawPixel(const azimuthRadiusSt object, const float xOffset, const float yOffset, const float rZ, const QColor color = Qt::white);

   robotFloor *rfloor = nullptr;
   linePoints *lPoints = nullptr;
   Ui::topViewer *ui = nullptr;
   mainWidget *parent = nullptr;

   detPosSt calcLast;
   QPen penDotLine;
};

#endif // TOPVIEWER_HPP
