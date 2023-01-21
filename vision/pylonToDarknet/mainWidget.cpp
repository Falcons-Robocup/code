// Copyright 2021-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// TODO list
// figure out of zerocopy makes sense: see
// https://manualzz.com/doc/1444308/pylon-4-camera-software-suite-for-linux-for-use

// frame grabber some stalling for a short moment, following message in dmesg:
// [ 1603.987070] xhci_hcd 0000:00:14.0: WARN Cannot submit Set TR Deq Ptr
// [ 1603.987072] xhci_hcd 0000:00:14.0: A Set TR Deq Ptr command is pending.
// [ 1603.990554] xhci_hcd 0000:00:14.0: bad transfer trb length 16384 in event trb



#include <QDebug>
#include <QDateTime>
#include <QTimer>
#include <QThread>

#include <unistd.h> // for usleep

// file:///opt/pylon/share/pylon/doc/C++/sample_code.html#sample_ParametrizeCamera_GenericParameterAccess
#include <pylon/PylonIncludes.h>

#include "detector.hpp"
#include "grabber.hpp"
#include "backend.hpp"
#include "mainWidget.hpp"
#include "ui_mainWidget.h"

using namespace GenApi;
using namespace Pylon;
using namespace std;

mainWidget::mainWidget(QWidget *parent) : QWidget(parent), ui(new Ui::mainWidget) {
   ui->setupUi(this);
   this->setWindowTitle("pylonToDarknet");

   startTime = QDateTime::currentDateTimeUtc();
   // qDebug().noquote() << "INFO    application start at UTC:" << startTime.toString("HH:mm:ss.zzz");

   // TODO: provide robot number through argument
   robot = 9; // WARNING: robot index starts at 1

   if( robot < 1 || robot > 15 ) {
      qDebug() << "ERROR   robot index" << robot << "out of range";
      exit(EXIT_FAILURE);
   }

#ifndef USE_JPG
   // TODO: move next initialization to grabber
   try {
      PylonInitialize();

      // Get the transport layer factory.
      CTlFactory& tlFactory = CTlFactory::GetInstance();

      // Get all attached devices and exit application if no device is found.
      DeviceInfoList_t devices;
      if ( tlFactory.EnumerateDevices(devices) == 0 ) {
         throw RUNTIME_EXCEPTION( "No camera present.");
      }

      // Create an array of instant cameras for the found devices
      cameras = new CInstantCameraArray( devices.size());

      // Create and attach all Pylon Devices.
      for ( size_t ii = 0; ii < cameras->GetSize(); ii++ ) {
         cameras->operator[](ii).Attach( tlFactory.CreateDevice( devices[ ii ]));
         cout << "using camera device " << ii << " of " << cameras->GetSize() << " " << cameras->operator[](ii).GetDeviceInfo().GetModelName();
         cout << " serial " << cameras->operator[](ii).GetDeviceInfo().GetSerialNumber() << endl;
      }

      // camera = new CBaslerUniversalInstantCamera( CTlFactory::GetInstance().CreateFirstDevice() ); // native camera interface
   } catch (const GenericException& err) {
      cerr << "An exception occurred." << endl << err.GetDescription() << endl;
   }
#endif

   conf = new config(cameras);
   conf->init();

   confDiag = new configDialog(this);
   topView = new topViewer(this);
   rFloor = new robotFloor(topView->topScene);
   topView->setup(lPoints, rFloor); // TODO: restructure to prevent topView and robotFloor dependency
   lPoints = new linePoints(robot);
   confDiag->setup();
   topView->show();
   int ageThreshHold = 0; // for testing directly provide the potential location, regardless of the age
   detPos = new determinePosition(lPoints, ageThreshHold, rFloor);

   for( size_t ii = 0; ii < CAMS; ii++ ) {
      camScenes[ii] = new QGraphicsScene(this);
      topScenes[ii] = new QGraphicsScene(this);
      frames[ii] = new frame(this, ii, robot);
   }

   ui->camView0->setScene(this->camScenes[0]);
   ui->camView0->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
   ui->camView0->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

   ui->camView1->setScene(this->camScenes[1]);
   ui->camView1->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
   ui->camView1->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

   ui->camView2->setScene(this->camScenes[2]);
   ui->camView2->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
   ui->camView2->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

   ui->camView3->setScene(this->camScenes[3]);
   ui->camView3->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
   ui->camView3->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

   ui->topView0->setScene(this->topScenes[0]);
   ui->topView0->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
   ui->topView0->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

   ui->topView1->setScene(this->topScenes[1]);
   ui->topView1->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
   ui->topView1->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

   ui->topView2->setScene(this->topScenes[2]);
   ui->topView2->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
   ui->topView2->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

   ui->topView3->setScene(this->topScenes[3]);
   ui->topView3->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
   ui->topView3->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

   imag = new imageTools(this); // create after creation of camScenes, TODO should this one for each camera?

   for( size_t ii = 0; ii < CAMS; ii++ ) {
      rgbImages[ii] = new QImage(CAM_WIDTH, CAM_HEIGHT, QImage::Format_RGB32);
      imag->createTestImage(*rgbImages[ii]);
   }

   conf->update();

   // grabber thread
   grabThread = new QThread;
   grab = new grabber(this);
   detect = new detector(this); // has dependency on grabber, create after grabber initialization, TODO: is this dependency still there?, i don't see it anymore, if not move to detector section
   grab->moveToThread(grabThread);
   // signal started() is emitted when the thread is executing
   connect(grabThread, SIGNAL(started()), grab, SLOT(process())); // start (once) the process method from the thread
   // signal finished() is emitted when the thread has finished excuting
   connect(grabThread, SIGNAL(finished()), grabThread, SLOT(deleteLater()));
   connect(grab, SIGNAL(finished()), grabThread, SLOT(quit()));
   connect(grab, SIGNAL(finished()), grab, SLOT(deleteLater()));
   grabThread->start(); // start the grabber thread, optional add priority parameter to start()

   // localization thread
   localizationThread = new QThread;
   localiztn = new localization(this);
   localiztn->moveToThread(localizationThread);
   connect(localizationThread, SIGNAL(finished()), localizationThread, SLOT(deleteLater()));
   connect(localiztn, SIGNAL(finished()), localizationThread, SLOT(quit()));
   connect(localiztn, SIGNAL(finished()), localiztn, SLOT(deleteLater()));
   localizationThread->start(); // start the localization thread

#ifdef NONO
   // detector thread
   connect(detect, SIGNAL(finished()), detect, SLOT(quit())); // TODO: remove
   connect(detect, SIGNAL(finished()), detect, SLOT(deleteLater()));
   detect->start(); // start the detector thread
#else
   detectThread = new QThread;
   detect->moveToThread(detectThread);
   connect(detectThread, SIGNAL(finished()), detectThread, SLOT(deleteLater()));
   connect(detect, SIGNAL(finished()), detectThread, SLOT(quit()));
   connect(detect, SIGNAL(finished()), detect, SLOT(deleteLater()));
   detectThread->start(); // start the detector thread
#endif

   // connect the grabber signal to the detector slot
   qRegisterMetaType<size_t>("size_t"); // size_t is not default available as meta type
   connect(grab, SIGNAL(grabReady(size_t)), detect, SLOT(process(size_t)) ); // start the detector when there is a new camera image
   connect(grab, SIGNAL(grabReady(size_t)), localiztn, SLOT(process(size_t)) ); // start the localization when there is a new camera image

   // backend thread
   backnd = new backend(this);
   connect(backnd, SIGNAL(started()), backnd, SLOT(init())); // initialize backend thread (so parent->backnd is assigned)
   connect(backnd, SIGNAL(finished()), backnd, SLOT(deleteLater()));
   backnd->start(); // start method run of the backend thread

   // connect the detector signal to the dewarp slot
   connect(detect, SIGNAL(detectReady(size_t)), backnd, SLOT(process(size_t)) ); // start the process method when a new frame with objects is available

   exitTimer = new QTimer(this);
   exitTimer->setInterval(3000); // in milli seconds
   connect( exitTimer, &QTimer::timeout, [=]() { exitTimerOccured(); } );
   // exitTimer->start();

   updateTimer = new QTimer(this);
   // updateTimer->setInterval(250); // 4 fps
   updateTimer->setInterval(100); // 10 fps
   // updateTimer->setInterval(10); // 100 fps
   connect( updateTimer, &QTimer::timeout, [=]() { updateTimerOccured(); } );
   updateTimer->start();
}

mainWidget::~mainWidget() {
   detect->stop();
   grab->stop();
   while( grab->busyStop() ) {
      // qDebug("wait for grabber done");
      usleep(10000);
   };
   // qDebug("grabber exit");

#ifdef NONO
   // TODO: enable again
   // with this section disabled, it looks like the camera's do not disapear anymore
   for( size_t ii = 0; ii < cameras->GetSize(); ii++ ) {
      GenApi::INodeMap& nodemap = cameras->operator[](ii).GetNodeMap();
      qDebug("INFO    reset camera %zu", ii);
      CCommandParameter(nodemap, "DeviceReset").Execute();
   }
#endif
   // qDebug("close cameras");
   cameras->Close();
   // TODO: find out if there is also a way to powerdown the camera (less power and heat)
   // qDebug("pylon terminate");
   PylonTerminate();

   // TODO: find out how to make use of thread wait() e.g. detectThread->wait();

   while( detect->getBusy() ) {
      qDebug("INFO    wait for detector done");
      usleep(10000);
   };
   // qDebug("detector exit");

   backnd->wait();

   delete ui;
   qDebug("INFO    program exit done");
}

void mainWidget::updateTimerOccured() {
   // qDebug() << "INFO    update timer occured";
   // to save CPU cycles, this updater runs on a lower rate then the grabber, decoder and backend

   topView->update();

   // NOTE: assume the available window size for all camera views is the same and for all top views is the same
   imag->showAllImages(ui->camView0->width(), ui->camView0->height(), ui->topView0->width(), ui->topView0->height());
   conf->update();
   confDiag->update();
   if( backnd->isRunning() ) {
      qDebug() << "WARNING backend is running";
   }
   if( ! backnd->isFinished() ) {
      qDebug() << "WARNING backend is not finished";
   }
}

void mainWidget::exitTimerOccured() {
   qDebug("INFO    the exit timer expired, stop application (expected behavior)");
   QCoreApplication::exit(EXIT_SUCCESS);
}

void mainWidget::on_exitButton_clicked() {
   qDebug("INFO    program exit request from exit button");
   QCoreApplication::exit(EXIT_SUCCESS);
}

void mainWidget::on_grabButton_clicked() {
   imag->saveQImageDate(*rgbImages[0], *rgbImages[1], *rgbImages[2], *rgbImages[3]);
}

void mainWidget::on_configButton_clicked() {
   confDiag->show();
}

void mainWidget::on_resetButton_clicked() {
   conf->reset();
   imag->reset();
}
