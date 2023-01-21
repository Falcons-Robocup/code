// Copyright 2021-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// for pylon camera grab examples see:
// file:///opt/pylon/share/pylon/doc/C++/sample_code.html

// to determine which features are supported by the camera, checkout:
// https://docs.baslerweb.com/features

#include <QApplication>
#include <QDebug>
#include <QDateTime>
#include <QRandomGenerator>

#include <strings.h> // bzero
#include <unistd.h> // usleep

#include "mainWidget.hpp"
#include "grabber.hpp"

// file:///opt/pylon/share/pylon/doc/C++/sample_code.html#sample_ParametrizeCamera_GenericParameterAccess
#include <pylon/PylonIncludes.h>
// #include <pylon/BaslerUniversalInstantCamera.h>

using namespace GenApi;
using namespace Pylon;
// using namespace Basler_UniversalCameraParams;
using namespace std;

// NOTE: there is only one grabber class that captures the data for all camera's

grabber::grabber(mainWidget *parent) {
   this->parent = parent;
   this->conf = parent->conf;
   this->cameras = parent->cameras;
   keepGoing = false;
   busyStopValue = false;

   for( size_t cam = 0; cam < CAMS; cam++ ) {
      frameCounter[cam] = 0;
      captureTime[cam] = QDateTime(QDate(1970, 1, 1), QTime(0, 0, 0));
      fps[cam] = 0.0;
      received[cam] = false;
      captureTimeIndex[cam] = 0;
      for( size_t ii = 0; ii < GRAB_FPS_SAMPLES; ii++ ) {
         captureTimeHistory[cam][ii] = 0;
      }
   }

#ifdef USE_JPG
   keepGoing = true;
#else
   try {
      for( size_t ii = 0; ii < cameras->GetSize(); ii++ ) {
         cameras->operator[](ii).MaxNumBuffer = 20; // default is 10, if to low (e.g. 1) then the following error occurs
         // ERROR   un-successful grab for cam 0, message: Payload data has been discarded. Payload data can be discarded by the camera device if the available bndwidth is insufficient.
         // with 5 only 35 fps can be achieved, with 20 around 37 fps can be achieved
         // qDebug().noquote() << "camera" << ii << "max number of buffers is"<< cameras->operator[](ii).MaxNumBuffer();
      }

      // TODO: move next section to process

      // continue grabbing frames
      // TODO: find out what capture strategie to use
      // file:///opt/pylon/share/pylon/doc/C++/group___pylon___instant_camera_api_generic.html#gga56a922b1bd37848234153b9e12f7fecbacb541053c64948f5d55e7955588deb33
      // file:///opt/pylon/share/pylon/doc/C++/group___pylon___instant_camera_api_generic.html#ggaf582b0a1fa8604d4e18857bd525291e6a1006c968199ded056843b9c42854b206
      // camera->StartGrabbing(GrabStrategy_LatestImages);
      cameras->StartGrabbing(GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByUser);

      keepGoing = true;
   } catch (const GenericException& e) {
      cerr << "ERROR   " << e.GetDescription() << endl;
      exit(1);
   }
#endif
}

grabber::~grabber() {
   // free resources
}

void grabber::stop() {
   // qDebug("grabber stop");
   keepGoing = false;
}

// Start processing data.
void grabber::process() {
   // allocate resources using new here
   busyStopValue = true;

#ifdef USE_JPG
   size_t cam = 0;
   while( keepGoing ) {
      captureTime[cam] = QDateTime::currentDateTimeUtc().addMSecs(-3); // TODO: calibrate shutter to software latency
      // read jpg image from disc
      // qDebug("INFO    start reading jpg file for cam %zu", cam);
      parent->imag->readJpg(parent->robot, cam);

      // start the detection by sending a signal to the detection class
      parent->detect->setNewImages(cam); // TODO: why this handshake? is the next emit not good enough?
      emit grabReady(cam); // start the detector and localization

      // update statistics
      frameCounter[cam]++;
      updateFps(cam);

      if( cam < CAMS) {
         cam++;
      } else {
         cam = 0;
      }

      if( cam >= CAMS ) { cam = 0; }
      usleep(200);
   }
#else
   while( keepGoing && cameras->IsGrabbing() ) {
      try {
         cameras->RetrieveResult( 5000, ptrGrabResult, TimeoutHandling_ThrowException);

         // Image grabbed successfully?
         if (ptrGrabResult->GrabSucceeded()) {
            // qDebug("INFO    grab succesfull %1d %1d %1d %1d",
            //        ptrGrabResult->GetWidth(), ptrGrabResult->GetHeight(), ptrGrabResult->GetOffsetX(), ptrGrabResult->GetOffsetY());

            // cout << "image size " << ptrGrabResult->GetImageSize() << " expected " << 1920*1200 << endl;;
            // EPixelType myType  = ptrGrabResult->GetPixelType();
            // cout << "bit depth " << BitDepth(myType) << " bits per pixel " << BitPerPixel(myType) << endl;

            size_t cam = ptrGrabResult->GetCameraContext();
            if( cam >= CAMS ) {
               cerr << "ERROR    cam index is " << cam << " while cam index is from 0 to " << CAMS - 1 << endl;
               exit(1);
            }
            captureTime[cam] = QDateTime::currentDateTimeUtc().addMSecs(-3); // TODO: calibrate shutter to software latency
#ifdef NONO
            received[cam] = true;
            bool complete = true;
            for( size_t ii = 0; ii < CAMS; ii++ ) {
               if( ! received[ii]) {
                  complete = false;
               }
            }
            if( complete ) {
               QString myString;
               // reset for the next frame from this camera
               for( size_t ii = 0; ii < CAMS; ii++ ) {
                  myString.append(QString().asprintf(" %6u", frameCounter[ii]));
                  received[ii] = false;
               }
               qDebug().noquote() << "INFO    at least one frame from all" << CAMS << "cameras" << myString;
            }
#endif

            const uint8_t *buff = (uint8_t *) ptrGrabResult->GetBuffer();
            // cout << "value of first pixel: " << (uint32_t) buff[0] << endl;

            // create pixels from bayer data and downscale from 1920x1200 to 960x600 or 1728x1216 to 864x608
            // TODO: direclty scale down to 416x416 and only do the conversion to 864x608 when needed
            // So move this function to detector and "viewer"

#ifdef USE_JPG
            (void) buff;
            parent->imag->readJpg(parent->robot, cam);
#else
            // the order of the camera's is determined by the serial, but this does not match with the mechanical mount
            // swap camera 1 and 3 to match with mechanical order
            // TODO: find better way of alignment between serial and mechanical order
            // WARNING: it might be that now the configuration of the camera does not match anymore with the video data
            if( cam == 1 ) {
               cam = 3;
            } else if( cam == 3 ) {
               cam = 1;
            }
            parent->imag->downscale_1_1_2(buff, cam); // TODO: move to one of the 4 downscale threads
#endif

            // start the detection by sending a signal to the detection class
            parent->detect->setNewImages(cam);
            emit grabReady(cam);

            // update statistics
            frameCounter[cam]++;
            updateFps(cam);
         } else {
            qDebug().noquote().nospace() << "ERROR   un-successful grab for cam " << ptrGrabResult->GetCameraContext() << ", message: " << ptrGrabResult->GetErrorDescription();
         } // grab succesfull
      } catch (const GenericException& e) {
         cerr << "ERROR   An exception occurred, message: " << e.GetDescription() << endl;
         exit(1);
      } // try
   } // while keepGoing

   cameras->StopGrabbing();
#endif

   emit finished(); // ready to cleanup thread, TODO: probably not required because signal finished() is already emitted when thread has finished
   // qDebug("grabber thread finished");
   busyStopValue = false;
}

// for each camera calculate fps from capture moment
void grabber::updateFps(size_t cam) {
   qint64 current = QDateTime::currentDateTime().toMSecsSinceEpoch();

   qint64 delta = current - captureTimeHistory[cam][captureTimeIndex[cam]];
   double average = 1.0 * delta / GRAB_FPS_SAMPLES; // in milli seconds
   fps[cam] = 1000.0 / average;

   // store current time for later usage
   captureTimeHistory[cam][captureTimeIndex[cam]] = current;

   // move the pointer to the next slot
   captureTimeIndex[cam]++;
   // wrap around
   if( captureTimeIndex[cam] == GRAB_FPS_SAMPLES ) {
      captureTimeIndex[cam] = 0;
   }
}
