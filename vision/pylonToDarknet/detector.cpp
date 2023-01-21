// Copyright 2021-2022 Andre Pool
// SPDX-License-Identifier: Apache-2.0

// #define DETECTOR_ENABLE

#include <QDateTime>
#include <QDebug>
#include <QImage>
#include <QVector>

#include <unistd.h> // usleep

#include "detector.hpp"
#include "mainWidget.hpp"

using namespace std;

detector::detector(mainWidget *parent) {
   this->parent = parent;

   // initialize used variables
   busy = false;
   for( size_t cam = 0; cam < CAMS; cam++ ) {
      newImages[cam] = false;
   }

#ifdef DETECTOR_ENABLE
   // NOTE load_network_custom also performs fuse_conv_batchnorm
   // load_network_custom(cfg_file_name, weights_file_name, clear, batch_size);
   net = load_network_custom((char *) configFile.c_str(), (char *) weightsFile.c_str(), 1, 1); // clear and batch size 1

   calculate_binary_weights(*net);
#endif

   srand(2222222); // set the seed, but is the random function used during detect?
   keepGoing = true;
}

//void detector::run() {
//    qDebug() << "INFO    detector run";
//}

void detector::stop() {
   // qDebug("detector stop");
   keepGoing = false;
}

// Start processing data.
void detector::process(size_t cam) {
   if( ! keepGoing ) { return; }
   busy = true;
   bool newImage;
   do {
      newImages[cam] = false; // new image used

      // perform the detection, this takes quite some time
#ifdef DETECTOR_ENABLE
      runDetector(cam);
#else
      runFakeDetector(cam);
#endif
      emit detectReady(cam); // inform dewarper that one frame with objects is available

      // TODO: apparantly this does not work expected, kind of works when printing is enabled (qDebug)
      // check if in the meantime a new image became available or if there is an image of another camera
      newImage = false;
#ifndef NONO
      for( size_t ii = 0; ii < CAMS; ii++ ) {
         size_t roundRobin = ( cam + ii + 1 ) % CAMS;
         if( newImages[roundRobin ]){
            newImage = true;
            ii = CAMS; // exit for loop
            cam = roundRobin;
            // qDebug("cam %zu round robin %zu", cam, roundRobin);
         }
      } // for
#endif
   } while( newImage && keepGoing );

   qDebug("INFO    leaving detector thread");
   busy = false;
   emit finished(); // ready to cleanup thread
}

void detector::runDetector(size_t cam) {
   image origImg = QImageToImage(*parent->rgbImages[cam]);
   image resizeImg = resize_image(origImg, net->w, net->h); // resize to 416x416

   // perform the object detection
   double timeStart = get_time_point();
   network_predict_ptr(net, (float *)resizeImg.data);
   float calcTime = ( get_time_point() - timeStart )/ 1000000.0; // scale from micro seconds to seconds

   // create bounding boxes (with relative size) for the detected objects
   int nboxes = 0;
   // get_network_boxes(net, width, height, threshold, hier_threshold, map, relative, num_boxes, letter);
   // NOTE: hier_threshold not used, custom_get_region_detections
   detection *dets = get_network_boxes(net, 1, 1, thresh, 0, 0, 1, &nboxes, 0);

   // TODO investigate if things below require GPU and if it makes sense to move them to the backend thread

   // one detected box can be classified for multiple classes, find the best class for each box
   diounms_sort(dets, nboxes, classes, nms_thresh, nms_kind, 0); // https://github.com/Zzh-tju/DIoU-darknet

   // copy the objects to a struct
   detectionToFrame(dets, nboxes, classes, calcTime, cam);

   free_detections(dets, nboxes);
   free_image(origImg);
   free_image(resizeImg);
}

// TODO convert direct from bayer to darknet image type
// TODO combine conversion with resize_image (416x416)
// TODO invest if it is required to move the image conversion to a separate thread (to maximize usage of GPU)
//   imageConversion thread

// Convert Qt image type to darknet image type
// darknet image type:
// red line
// green line
// blue line
// ..
// red line
// green line
// blue line
image detector::QImageToImage(const QImage input) {
   int ww = input.width();
   int hh = input.height();
   int cc = 3;
   image im = make_image(ww, hh, cc);
   for (int yy = 0; yy < hh; ++yy) {
      for (int kk = 0; kk < cc; ++kk) {
         for (int xx = 0; xx < ww; ++xx) {
            QRgb rgb = input.pixel(xx,yy);
            int color;
            if( kk == 0 ) {
               color = (rgb >> 16 ) & 0xff; // red
            } else if( kk == 1 ) {
               color = (rgb >> 8 ) & 0xff; // green
            } else {
               color = (rgb >> 0 ) & 0xff; // blue
            }
            im.data[kk*ww*hh + yy*ww + xx] = color / 255.0f;
         }
      }
   }
   return im;
}

// convert the yolo / darknet objects to the "frame" / "object" type
void detector::detectionToFrame(detection *dets, int nboxes, int classes, float calcTime, size_t cam) {
   mutex[cam].lock();

   this->calcTime[cam] = calcTime;
   captureTime[cam] = parent->grab->getCaptureTime(cam);
   frameCounter[cam] = parent->grab->getFrameCounter(cam);

   objects[cam].clear();
   for (int ii = 0; ii < nboxes; ++ii) {
      for (int jj = 0; jj < classes; ++jj) {
         if (dets[ii].prob[jj] > 0.005) { // function get_network_boxes() has already filtered dets by actual threshold
            object obj;
            obj.classId = (quint32)jj;
            obj.xCenter = dets[ii].bbox.x;
            obj.yCenter = dets[ii].bbox.y;
            obj.width = dets[ii].bbox.w;
            obj.height = dets[ii].bbox.h;
            obj.confidence = dets[ii].prob[jj];
            obj.valid = true;
            objects[cam].push_back(obj);
         }
      }
   }
   mutex[cam].unlock();
}

// create a frame with fake objects (testing only)
void detector::runFakeDetector(size_t cam) {
   mutex[cam].lock();

   frameCounter[cam] = parent->grab->getFrameCounter(cam);
   captureTime[cam] = parent->grab->getCaptureTime(cam);

   double timeStart = get_time_point();
   usleep(100000); // emulate the detection processing time
   calcTime[cam] = float(( get_time_point() - timeStart ) / 1000000.0); // scale from micro seconds to seconds

   objects[cam].clear();
   object obj;

   obj.classId = 0; // ball
   obj.xCenter = 0.60;
   obj.yCenter = 0.48;
   obj.width = 0.29;
   obj.height = 0.27;
   obj.confidence = 0.90;
   obj.valid = true;
   objects[cam].push_back(obj);

#ifdef NONO

   float offset = ( frameCounter[cam]>>4 & 0x1f ) * 0.01;
   obj.classId = 0; // ball
   obj.xCenter = 0.35 - offset;
   obj.yCenter = 0.76 - offset;
   obj.width = 0.10;
   obj.height = 0.12;
   obj.confidence = 0.56;
   obj.valid = true;
   objects[cam].push_back(obj);
   obj.classId = 1; // obstacle
   obj.xCenter = 0.0 + cam * 0.15;
   // obj.yCenter = 0.40 + cam * 0.12;
   obj.yCenter = 0.571;
   obj.width = 0.01;
   obj.height = 0.01;
   obj.confidence = 0.71;
   obj.valid = true;
   objects[cam].push_back(obj);

   obj.classId = 2; // human
   obj.xCenter = 0.76;
   obj.yCenter = 0.48;
   obj.width = 0.29;
   obj.height = 0.27;
   obj.confidence = 0.30;
   obj.valid = true;
   objects[cam].push_back(obj);

   obj.classId = 3; // border
   obj.xCenter = 0.17;
   obj.yCenter = 0.52;
   obj.width = 0.15;
   obj.height = 0.10;
   obj.confidence = 0.41;
   obj.valid = true;
   objects[cam].push_back(obj);

   obj.classId = 4; // goal post
   obj.xCenter = 0.90;
   obj.yCenter = 0.60;
   obj.width = 0.12;
   obj.height = 0.15;
   obj.confidence = 0.25;
   obj.valid = true;
   objects[cam].push_back(obj);
#endif
   mutex[cam].unlock();
}

void detector::getFrame(size_t cam, float &calcTime, QDateTime &captureTime, quint32 &frameCounter, QVector<object> &objects) {
   mutex[cam].lock();
   calcTime = this->calcTime[cam];
   captureTime = this->captureTime[cam];
   frameCounter = this->frameCounter[cam];
   objects = this->objects[cam];
   mutex[cam].unlock();
}
