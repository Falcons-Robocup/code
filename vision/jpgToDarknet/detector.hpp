// Copyright 2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef DETECTOR_HPP
#define DETECTOR_HPP

#include <QByteArray>
#include <QImage>

#include <cstdio>
#include <string>
#include <vector>

#include <darknet.h>

typedef struct {
    uint32_t class_id;
    float x;
    float y;
    float w;
    float h;
    float confidence;
} object_t;

typedef struct {
    uint32_t frame_id;
    float calcTime; // in seconds
    std::vector<object_t> obj;
} frame_t;

class detector {
public:
    detector(QImage *rgbImage= nullptr);
    void update(const QByteArray jpegData);
    void updateQimage();
    void updateImage( image origImg );
    void updateGrabPtr(uint8_t *grabBuff = nullptr);
    frame_t getObjects( ){ return objects; }

private:
    const int classes = 5; // net->layers[last yolo layer].classes (../robocup_ml/yolov4.cfg)
    const std::string configFile = "/home/robocup/robocup_ml/yolov4.cfg";
    network *net;
    const std::string namesFile = "/home/robocup/robocup_ml/obj.names";
    const NMS_KIND nms_kind = GREEDY_NMS; // net->layers[last yolo layer].nms_kind (../robocup_ml/yolov4.cfg)
    const float nms_thresh = 0.45; // threshold related to intersection over union, default value demo
    frame_t objects;
    const float thresh = 0.2; // threshold to accept object, default value run_detector 0.25
    const std::string weightsFile =  "/home/robocup/darknet/yolov4_final.weights_20201112";
    QImage *rgbImage;

    frame_t detectionToStruct(detection *dets, int nboxes, int classes, double calcTime);
    void printObjects(const frame_t objects);
    image QImageToImage(const QImage input);
    image resize_image2(uint8_t *grabBuff, image im, int w, int h);
    float get_pixel2(image m, int x, int y, int c);
    void set_pixel2(image m, int x, int y, int c, float val);
    void add_pixel2(image m, int x, int y, int c, float val);
    float get_pixel3(uint8_t *grabBuff, int x, int y, int c);


};

#endif // DETECTOR_HPP
