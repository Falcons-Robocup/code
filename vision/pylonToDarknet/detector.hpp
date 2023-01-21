// Copyright 2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef DETECTOR_HPP
#define DETECTOR_HPP

#include <QDateTime>
#include <QImage>
#include <QMutex>
#include <QObject>
// #include <QThread>
#include <QVector>

#include <cstdio>
#include <string>

#include <darknet.h>

#include "config.hpp"
#include "object.hpp"

class mainWidget; //forward declaration

// class detector : public QThread {
class detector : public QObject {
   Q_OBJECT

public:
   explicit detector(mainWidget *parent);
   void getFrame(size_t cam, float &calcTime, QDateTime &captureTime, quint32 &frameCounter, QVector<object> &objects);
   void setNewImages(size_t cam) { newImages[cam] = true; } // handshake from grabbers

public slots:
   bool getBusy() { return busy; }
   void process(size_t cam);
   void stop();

signals:
   void detectReady(size_t cam); // inform dewarp objects available
   void finished(); // used to cleanup thread, is required because used as signal

private:
   // void run() override; // -> run once when ->start() is called
   void detectionToFrame(detection *dets, int nboxes, int classes, float calcTime, size_t cam);
   image QImageToImage(const QImage input);
   void runDetector(size_t cam);
   void runFakeDetector(size_t cam);

   bool busy;
   float calcTime[CAMS];
   QDateTime captureTime[CAMS];
   quint32 frameCounter[CAMS];
   QVector<object> objects[CAMS];
   QMutex mutex[CAMS];
   mainWidget *parent = nullptr;
   bool keepGoing;
   bool newImages[CAMS]; // handshake between grabber and detector

   // yolo / darknet related variables
   const int classes = 5; // net->layers[last yolo layer].classes in yolov4.cfg
   const std::string configFile = "/home/robocup/robocup_ml/yolov4.cfg";
   network *net;
   const std::string namesFile = "/home/robocup/robocup_ml/obj.names";
   const NMS_KIND nms_kind = GREEDY_NMS; // net->layers[last yolo layer].nms_kind (../robocup_ml/yolov4.cfg)
   const float nms_thresh = 0.45; // threshold related to intersection over union, default value demo
   const float thresh = 0.2; // threshold to accept object, default value run_detector 0.25
   const std::string weightsFile =  "/home/robocup/darknet/yolov4_final.weights_20201112";
};

#endif // DETECTOR_HPP
