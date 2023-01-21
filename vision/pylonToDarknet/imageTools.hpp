// Copyright 2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef IMAGE_TOOLS_HPP
#define IMAGE_TOOLS_HPP

#include <QByteArray>
#include <QColor>
#include <QFont>
#include <QGraphicsScene>
#include <QImage>
#include <QWidget>

#include "config.hpp"
#include "detector.hpp"
#include "object.hpp"

class mainWidget; //forward declaration

class imageTools {
public:
   // TODO: cleanup not used methods
   // TODO: move to private if applicable
   explicit imageTools(mainWidget *parent = nullptr);
   void reset();
   void downscale_1_1_2(const uint8_t *buff = nullptr, size_t cam = 0);
   QImage rotate(const QImage origImage);
   void rotate(QByteArray &jpegData);
   void save(QByteArray jpegData, QString filename);
   void showAllImages(qreal camWidth, qreal camHeight, qreal topWidth, qreal topHeight);
   void showCam(size_t cam);
   void showTop(size_t cam);
   QImage convertArrayToImage(QByteArray myArray);
   void createTestImage(QImage &myImage);
   void createTestImagePtr(uint8_t *imagePtr);
   void readJpg(QImage &myImage, size_t frameCounter);
   void readJpg(int robot, size_t camera); // testing only
   void addMovingLine(QImage &rgbImage, size_t frameCounter);
   void saveQImageDate(QImage image0, QImage image1, QImage image2, QImage image3);


#ifdef NONO
   // only used for testing
   void full_frame_1_1_1(const uint8_t *buff = nullptr, size_t cam = 0);
   void interpolate_2_2_5(const uint8_t *buff = nullptr, size_t cam = 0);
   void interpolate_5_5_6(const uint8_t *buff = nullptr, size_t cam = 0);
#endif

private:
   void addHeader(QGraphicsScene *scene);
   void addCamObject(QGraphicsScene *scene, object obj);
   void addTopObject(QGraphicsScene *scene, object obj);
   QImage readImage(QString fileName); // testing only

   int borderWidth;
   int borderHeight;
   QPen penGrid, penAxis, penRobot;
   QFont myFont;
   mainWidget *parent = nullptr;
   qreal camWidth, camHeight, topWidth, topHeight;
   quint32 frameCounter;
   float fpgaTemp;
   float grabberFps;
   float detectorFps;
   size_t linePoints;
};

#endif // IMAGE_TOOLS_HPP

