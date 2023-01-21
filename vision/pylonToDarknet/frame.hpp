// Copyright 2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef FRAME_HPP
#define FRAME_HPP

#include <QDateTime>
#include <QMutex>
#include <QString>
#include <QVector>

#include "object.hpp"
#include "dewarp.hpp"

class mainWidget; //forward declaration

class frame {
public:
   explicit frame(mainWidget *parent, size_t camera, int robot);
   void getFrame(float &calcTime, QDateTime &captureTime, quint32 &frameCounter, QVector<object> &objects);
   double getFps() { return fps; }
   QString getPrintString(bool shortName);
   void print(bool shortname);
   void update();


private:
   void rangeCheck();
   void updateFps();

   float calcTime; // in seconds
   size_t camera;
   QDateTime captureTime;
#define DET_FPS_SAMPLES 20
   qint64 captureTimeHistory[DET_FPS_SAMPLES];
   size_t captureTimeIndex;
   Dewarper *dewarpLookup = nullptr; // TODO rename to dewarp
   double fps;
   quint32 frameCounter;
   QMutex mutex;
   QVector<object> objects;
   mainWidget *parent = nullptr;
   int robot;
   bool valid;
};

#endif // FRAME_HPP
