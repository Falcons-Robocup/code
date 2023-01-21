// Copyright 2021-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include <unistd.h> // for usleep

#include <QDateTime>
#include <QDebug>
#include <QImage>
#include <QFile>
#include <QByteArray>

#include <cfloat> // for float defines

#include "backend.hpp"
#include "exporter.hpp"
#include "mainWidget.hpp"

using namespace std;


backend::backend(mainWidget *parent) {
   this->parent = parent;
}

backend::~backend() {
   // TODO: why not called at end
   qDebug() << "backend destructor";
}

// WARNING: the default QThread::run() actually calls event loop QThread::exec(), which does not return
void backend::run() {
   qDebug() << "INFO    backend run, parent->backnd" << parent->backnd << "parent->backnd->thread()" << parent->backnd->thread();
   // for some reason the following does not work: QObject: Cannot create children for a parent that is in a different thread.
   // diagnstcs = new diagnostics(parent);
   // exprt = new exporter(parent);

   // exec(); // requires QThread::quit() or QThead::exit() to quit event loop
}

// run once after startup
void backend::init() {
   qDebug() << "INFO    backend init, parent->backnd" << parent->backnd << "parent->backnd->thread()" << parent->backnd->thread();
   // diagnostics and exporter require parent->backnd, which is not yet set in the backend constructor
   diagnstcs = new diagnostics(parent);
   exprt = new exporter(parent);

   // initialize used variables
   busy = false;
}

// triggered when detector has calculated a frame with objects
void backend::process(size_t camera) {
   busy = true;
   // qDebug() << "INFO    backend process started";

   parent->frames[camera]->update(); // also performs the dewarp
   exprt->update(camera);
   diagnstcs->update(camera); // TODO: send only 2 times per second
   printObjectsInterval();

#define ADDITIONAL_ROBOT
#ifdef ADDITIONAL_ROBOT
   quint8 robotBackup = parent->robot;
   parent->robot = 2;
   diagnstcs->update(camera);
   parent->robot = robotBackup;
#endif

   busy = false;
}

void backend::printObjectsInterval() {
   static qint64 previousTime = QDateTime::currentDateTime().toMSecsSinceEpoch();
   qint64 current = QDateTime::currentDateTime().toMSecsSinceEpoch();

   qint64 delta = current - previousTime;

   if( delta > 5000 ) { // delta in milli seconds
      previousTime = current;

      for( size_t ii = 0; ii < CAMS; ii++ ) {
         parent->frames[ii]->print(true);
      }
   }
}
