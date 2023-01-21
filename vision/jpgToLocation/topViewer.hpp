// Copyright 2021-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef TOPVIEWER_HPP
#define TOPVIEWER_HPP

#include <QGraphicsScene>
#include <QPen>
#include <QWidget>

#include "config.hpp"
#include "determinePosition.hpp"
#include "robotFloor.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class mainWidget; }
QT_END_NAMESPACE

class mainWidget; // forward declaration

class topViewer {
public:
   topViewer(Ui::mainWidget *ui);
   void setup(linePoints *lPoints, robotFloor *rFloor);
   // void drawCamera(int camera, QVector<objectSt> objects, int robot, float xOffset, float yOffset);
   // void drawRobot(int robot, float xOffset, float yOffset);
   void post(quint64 captureLates);
   void pre();

   void addStatus(const detPosSt calc);
   void drawRobot(const float x, const float y, const float rz, const QColor color = Qt::red);
   void drawPixel(const azimuthRadiusSt object, const float xOffset, const float yOffset, const float rZ, const QColor color = Qt::white);
   void showPosition(const detPosSt calc);
   QGraphicsScene *topScene = nullptr; // used by robotfloor

private:
   linePoints *lPoints = nullptr;
   robotFloor *rfloor = nullptr;
   Ui::mainWidget *ui = nullptr;

   detPosSt calcLast;
   QPen penDotLine;
};

#endif // TOPVIEWER_HPP



