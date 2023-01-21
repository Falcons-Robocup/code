// Copyright 2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef GRABBER_HPP
#define GRABBER_HPP

#include <QDateTime>
#include <QObject>

#include <pylon/PylonIncludes.h>
// WARNING next line requires QMAKE_CXXFLAGS += -Wno-deprecated-copy
// #include <pylon/BaslerUniversalInstantCamera.h>

#include "config.hpp"

class mainWidget; //forward declaration

class grabber : public QObject {
   Q_OBJECT

public:
   explicit grabber(mainWidget *parent);
   ~grabber();
   double getFps(size_t cam) { return fps[cam]; }
   quint32 getFrameCounter(size_t cam) { return frameCounter[cam]; }
   QDateTime getCaptureTime(size_t cam) { return captureTime[cam]; }

public slots:
   void process();
   void stop();
   bool busyStop() {
      return busyStopValue;
   }

signals:
   void finished(); // used to cleanup thread
   void grabReady(size_t cam); // inform decoder new data available

private:
   void updateFps(size_t cam);

   mainWidget *parent = nullptr;
   config *conf = nullptr;

   Pylon::CGrabResultPtr ptrGrabResult; // smart pointer containing the grab results, TODO: move declaration to process method

   Pylon::CInstantCameraArray *cameras = nullptr; // generic camera interface array
   // Pylon::CBaslerUniversalInstantCamera *camera; // native camera interface

   bool busyStopValue;
   bool keepGoing; // TODO: move declaration to process method
   bool received[CAMS];

#define GRAB_FPS_SAMPLES 100
   qint64 captureTimeHistory[CAMS][GRAB_FPS_SAMPLES];
   size_t captureTimeIndex[CAMS];
   double fps[CAMS]; // 0.1 to 1.000.000 (with BayerRG8, usb3 and 1920x1200 maximal 130 fps)
   quint32 frameCounter[CAMS];
   QDateTime captureTime[CAMS];
};

#endif // GRABBER_HPP
