// Copyright 2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <QDebug>

#include "mainWidget.hpp"
#include "frame.hpp"

frame::frame(mainWidget *parent, size_t camera, int robot) {
   this->parent = parent;
   this->camera = camera;
   this->robot = robot;

   // initialize used variables
   calcTime = 0.0;
   captureTime = QDateTime(QDate(1970, 1, 1), QTime(0, 0, 0));
   for( size_t ii = 0; ii < DET_FPS_SAMPLES; ii++ ) {
      captureTimeHistory[ii] = 0;
   }
   captureTimeIndex = 0;
   fps = 0.0;
   frameCounter = 0;
   objects.clear();
   valid = false;

   dewarpLookup = new Dewarper(robot, camera, false);
   if( CAM_WIDTH != dewarpLookup->getPixelWith() ) {
      qDebug() << "ERROR   cam" << camera << "width of" << CAM_WIDTH << "does not match with dewarp table width of" << dewarpLookup->getPixelWith();
   }
   if( CAM_HEIGHT != dewarpLookup->getPixelHeight() ) {
      // TODO enable again
      // qDebug() << "ERROR   cam" << camera << "height of" << HEIGHT << "does not match with dewarp table height of" << dewarpLookup->getPixelHeight();
   }
}

void frame::getFrame(float &calcTime, QDateTime &captureTime, quint32 &frameCounter, QVector<object> &objects) {
   mutex.lock(); // provide as one set
   calcTime = this->calcTime;
   captureTime = this->captureTime;
   frameCounter = this->frameCounter;
   objects = this->objects;
   mutex.unlock();
}

// get the frame, perform checkings and perform dewarp
void frame::update() {
   mutex.lock(); // keep all data as one set
   parent->detect->getFrame(camera, calcTime, captureTime, frameCounter, objects);
   valid = true;
   rangeCheck();
   updateFps();
   for( int ii = 0; ii < objects.size(); ii++ ) {
      objects[ii].update( (int)camera, dewarpLookup, robot );
   }
   mutex.unlock();
}

void frame::rangeCheck() {
   if( ! valid ) { return; }

   // frame counter should not exceed 2 days @ 40 FPS (typically it only runs a few hours)
   if( frameCounter >= 2*24*60*60*40 ) {
      qDebug() << "ERROR   frame id" << frameCounter << "out of range";
      valid = false;
   }

   if ( calcTime < 0.01 || calcTime >= 1.0 ) {
      qDebug() << "ERROR   calculation time" << calcTime << "out of range";
      valid = false;
   }

   QDateTime currentTime = QDateTime::currentDateTimeUtc();
   if ( captureTime > currentTime || captureTime > currentTime.addDays(2) || captureTime < currentTime.addDays(-2)) { // do not expect the application will run for more then 2 days
      qDebug().noquote() << "INFO    current UTC" << currentTime.toString("yyyy:MM:dd_HH:mm:ss.zzz");
      qDebug().noquote() << "ERROR   capture UTC" << captureTime.toString("yyyy:MM:dd_HH:mm:ss.zzz") << "out of range";
      valid = false;
   }
}

QString frame::getPrintString(bool shortName) {
   QString text;
   mutex.lock();
   for( int ii = 0; ii < objects.size(); ii++ ) {
      if ( ii == 0 ) {
         text.append( QString().asprintf("cam %1zu frame %7u calc %3.0fms ", camera, frameCounter, 1000.0 * calcTime));
      } else {
         text.append( "                               ");
      }

      text.append( objects[ii].getPrintString(shortName) );
      if( ii != objects.size() - 1 ) {
         text.append("\n");
      }
   }
   mutex.unlock();
   return text;
}

void frame::print(bool shortname = true) {
   if( ! valid ) { return; }
   qDebug().noquote() << getPrintString(shortname);
}

// use the time detection calculation time to calculate the fps
// Note: this method will no be called from the detector thread but from the dewarper thread
// (which is called for every detection)
// Note: the detection fps cannot exceed the capture fps, because the detection
// will only start when a new frame is captured
void frame::updateFps() {
   qint64 current = QDateTime::currentDateTime().toMSecsSinceEpoch();

   qint64 delta = current - captureTimeHistory[captureTimeIndex];
   double average = 1.0 * delta / DET_FPS_SAMPLES; // in milli seconds
   fps = 1000.0 / average;

   // store current time for later usage
   captureTimeHistory[captureTimeIndex] = current;

   // move the pointer to the next slot
   captureTimeIndex++;
   // wrap around
   if( captureTimeIndex == DET_FPS_SAMPLES ) {
      captureTimeIndex = 0;
   }
}
